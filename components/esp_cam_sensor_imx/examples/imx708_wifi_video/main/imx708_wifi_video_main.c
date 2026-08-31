/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMX708 live video over WiFi: camera -> hardware H.264 -> HTTP, continuously.
 *
 *   GET  /                  a page that plays the stream
 *   GET  /info              geometry, frame rate and codec string, as JSON
 *   GET  /stats             encoder and link counters, as text
 *   GET  :81/stream.mp4     live fragmented MP4 - what a browser plays
 *   GET  :81/stream.h264    live Annex-B elementary stream - what ffplay plays
 *
 * This is imx708_video and imx708_wifi_snapshot joined up, and the join is the
 * interesting part. imx708_video proved the encoder: 1080p28 at 9.2 ms mean
 * encode against a 35.7 ms budget, recorded into PSRAM and shipped down the
 * console afterwards because 2 Mbaud could not carry it live.
 * imx708_wifi_snapshot proved the radio: 6-7 Mbit/s sustained through
 * SDIO -> C6 -> air. That measurement is what made this example possible -
 * ~260 KB per JPEG puts MJPEG at about 3 fps on the same link, while H.264 at
 * 3 Mbit/s is 375 KB/s and the whole 28 fps fits with room over.
 *
 * The two halves are decoupled by a ring of encoded frames in PSRAM. The
 * encoder never waits for the network - it overwrites the oldest frame instead
 * - and a reader that falls behind is jumped forward to the newest IDR rather
 * than being allowed to drag the pipeline down with it. That is the difference
 * between live video and a file transfer: bounded latency matters more than
 * delivering every frame, and a stream that buffers in order to keep up is a
 * stream that is no longer live.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_http_server.h"

#include "esp_h264_enc_single_hw.h"
#include "esp_h264_alloc.h"

#include "imx_wifi.h"
#include "imx_fmp4.h"

/* ---- Camera pins (Waveshare ESP32-P4-WIFI6) ----------------------------- */
#define CAM_SCCB_I2C_PORT   0
#define CAM_SCCB_SCL_PIN    8
#define CAM_SCCB_SDA_PIN    7
#define CAM_SCCB_FREQ_HZ    100000
#define CAM_RESET_PIN       (-1)
#define CAM_PWDN_PIN        (-1)

#define CAM_DEV_PATH        ESP_VIDEO_MIPI_CSI_DEVICE_NAME

/*
 * Three capture buffers, as in imx708_video: every frame goes through the
 * encoder before its buffer can go back, so there is real work between DQBUF
 * and QBUF and the third buffer absorbs the jitter. At YUV420 each one is
 * w*h*3/2 = 3.1 MB of the 32 MB PSRAM.
 */
#define BUFFER_COUNT        3

/* ---- Encoder ------------------------------------------------------------ */

/* The sensor's only mode. Sets the encoder's bit budget per frame and the SPS
 * timing; it does not make frames arrive faster. */
#define VIDEO_FPS           28

/*
 * One IDR per second.
 *
 * Here the GOP is not only a quality knob, as it was in imx708_video: it is
 * also how long a new viewer waits. A stream can only be joined at an IDR, so
 * the first picture appears up to one GOP after the browser connects. Halving
 * it halves that wait and spends more of the bitrate re-sending the scene.
 */
#define VIDEO_GOP           VIDEO_FPS

/*
 * 3 Mbit/s = 375 KB/s.
 *
 * imx708_video used 4 Mbit/s because the link was irrelevant there: the clip
 * was recorded first, and the bitrate only decided how long the transfer took
 * afterwards. Live, the link is the constraint, and the measurement to respect
 * is the *worst* run rather than the best - /bench gave 7.2 Mbit/s once and
 * 4.8 Mbit/s on an identical second run with nothing changed. 3 Mbit/s sits
 * inside the slow run with headroom for whatever made it slow.
 */
#define VIDEO_BITRATE       3000000

/*
 * Encode one frame in every VIDEO_FRAME_SKIP the camera delivers.
 *
 * **This does not save bandwidth.** Rate control's budget is bits per second,
 * so halving the frame rate does not halve the bitrate - it gives each
 * remaining frame twice as many bits. The stream still costs VIDEO_BITRATE.
 * That is worth stating plainly because it is the obvious thing to reach for
 * when a link is too slow, and it is the wrong lever: VIDEO_BITRATE is the
 * bandwidth knob, and this is a quality-and-headroom one.
 *
 * What it does buy is chip time. The encoder needs 32-36 ms for a 1920x1072
 * frame - measured with the camera stopped and one frame fed to it repeatedly,
 * so that is the encoder's own cost, not contention - against a 35.7 ms frame
 * interval. At skip 1 the encoder is busy essentially all the time, and the
 * pipeline runs at 27.2 fps rather than 28 because of it.
 *
 * Default 1 all the same, because that headroom turned out not to be needed:
 * /bench at skip 0 (encoder stopped), 1, 2 and 4 gave 1.94, 2.99, 0.94 and
 * 3.5 Mbit/s - no relationship to the encoder at all, only the link's own
 * variance. A busy encoder does not slow the radio down.
 *
 * /skip?n= changes it at runtime, which is how that table was measured: one
 * boot, one scene, one association. Two of those three move enough between
 * reboots to swamp the effect being looked for.
 */
#define VIDEO_FRAME_SKIP    1

/* QP bounds for rate control. 51 is the codec's maximum (worst quality). */
#define VIDEO_QP_MIN        20
#define VIDEO_QP_MAX        45

/*
 * Per-frame encoder output. Must fit the largest IDR; the encoder reports
 * ESP_H264_ERR_OVERFLOW rather than scribbling past the end, but an overflowed
 * frame is a lost frame. This also sizes the per-viewer buffers below, so it is
 * not free - see the note there.
 */
#define ENC_OUT_BYTES       (512 * 1024)

/* See "1072 lines, not 1080" in imx708_video's README - unchanged here. */
#define ENCODE_16_ALIGNED   1

/*
 * AF_DEBUG_LOG: the AF algorithm's own per-scan-point logging. Useful while the
 * focus search runs at startup, noisy afterwards, and it is the only view of
 * what the search decided - the ISP AF statistics never reach the application.
 */
#define AF_DEBUG_LOG        0

/*
 * ENCODER_BITRATE_SWEEP: at boot, time the encoder at a range of rate-control
 * targets before the radio comes up. Off by default - it delays the stream by a
 * few seconds - but it is how the bitrate/encode-time curve below was measured,
 * and the way to re-measure it on a different scene.
 */
#define ENCODER_BITRATE_SWEEP 0

/* ---- Ports --------------------------------------------------------------- */
/*
 * Two servers, deliberately, and the page belongs to the streaming one.
 *
 * esp_http_server runs one task per instance and serves one request at a time
 * in it, so a handler that never returns - which is exactly what a live stream
 * is - owns the whole server for as long as someone is watching. On a single
 * instance the page would load, start the stream, and then nothing else would
 * ever be answered: no /stats while streaming, which is the one moment the
 * counters are worth reading.
 *
 * The split is the obvious fix; which side the *page* goes on is the part worth
 * thinking about. Serving it from the control port makes the stream
 * cross-origin, and while an Access-Control-Allow-Origin header covers that for
 * an ordinary browser, it is one more thing between a viewer and a picture -
 * measured here: an embedded browser refused the fetch outright with
 * ERR_BLOCKED_BY_CLIENT, and the page sat on "starting..." with nothing wrong
 * on the board. Serving the page from the stream port instead makes the fetch
 * same-origin and the question disappears.
 *
 * So: port 81 is the viewer - page, /info and the streams, all one origin, and
 * everything it needs is a relative URL. Port 80 is the instruments - /stats,
 * /bench and /skip, answerable at any moment because nothing there ever blocks.
 * The page links to them as plain navigations, which no origin rule touches.
 */
#define CTRL_PORT           80
#define STREAM_PORT         81

/* Stringify for the page, which needs the control port as JavaScript text. */
#define IMX_STR_(x)         #x
#define IMX_STR(x)          IMX_STR_(x)

/* ---- The frame ring ------------------------------------------------------ */
/*
 * 1 MB, about 2.8 s of encoded video at VIDEO_BITRATE.
 *
 * This is a latency budget, not a buffer to be filled. A reader more than this
 * far behind has its frames overwritten and is jumped to the live edge, so the
 * ring size is the most lateness the stream tolerates before it drops rather
 * than lags. Larger would hide a slow link by adding delay, which for live
 * video is the wrong trade.
 */
#define RING_BYTES          (1024 * 1024)

/* Descriptors, sized so the byte ring always runs out first: 256 frames is 9 s
 * at 28 fps, well past the ~2.8 s the bytes hold. */
#define RING_FRAMES         256

typedef struct {
    uint64_t start;     /*!< Absolute write position, so staleness is a subtraction. */
    uint32_t offset;    /*!< Where in s_ring the frame actually sits. */
    uint32_t len;
    uint32_t pts_ms;    /*!< Capture time, from the first frame. */
    bool     is_idr;
} ring_frame_t;

static uint8_t *s_ring;
static ring_frame_t s_desc[RING_FRAMES];
static uint64_t s_wpos;             /* bytes ever written, wrap padding included */
static uint32_t s_wseq;             /* frames ever published; the next seq to assign */
static SemaphoreHandle_t s_ring_lock;

/* ---- What the stream is -------------------------------------------------- */

static uint32_t s_w, s_enc_h;
static uint8_t s_skip = VIDEO_FRAME_SKIP;
static uint32_t s_bitrate = VIDEO_BITRATE;   /* what rate control is aiming at now */
static esp_h264_enc_param_hw_handle_t s_enc_param;
static char s_codec[16];            /* "avc1.42C029", once the first IDR has been seen */
static uint8_t s_init_seg[IMX_FMP4_INIT_MAX];
static size_t s_init_len;

/* ---- Counters ------------------------------------------------------------ */

static struct {
    uint32_t encoded;
    uint32_t idr;
    uint32_t failed;
    uint32_t enc_us_max;
    uint64_t enc_us_total;
    uint32_t dq_us_max;
    uint64_t dq_us_total;           /* time blocked waiting for the camera */
    uint32_t send_us_max;
    uint64_t send_us_total;         /* time blocked in the socket, per frame */
    uint32_t send_frames;
    uint64_t bytes;                 /* encoder output, over the whole run */
    int64_t  t_start;
    int32_t  viewers;
    uint32_t resyncs;               /* readers jumped forward, over all connections */
    uint32_t oversize;              /* frames skipped as too big for a viewer buffer */
    uint32_t skipped;               /* frames dropped before the encoder, by s_skip */
    uint64_t sent;                  /* payload bytes handed to the sockets */
} s_stat;

static const char *TAG = "imx708_wifi_video";

static const esp_video_init_csi_config_t csi_config[] = {{
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = { .port = CAM_SCCB_I2C_PORT, .scl_pin = CAM_SCCB_SCL_PIN, .sda_pin = CAM_SCCB_SDA_PIN },
        .freq = CAM_SCCB_FREQ_HZ,
    },
    .reset_pin = CAM_RESET_PIN,
    .pwdn_pin  = CAM_PWDN_PIN,
}};

/*
 * The autofocus VCM (DW9807, I2C 0x0c) is a separate chip from the sensor, so
 * esp_video probes for it separately and needs its own entry - without one the
 * motor auto-detect array is never walked and the lens is never driven.
 */
static const esp_video_init_cam_motor_config_t motor_config[] = {{
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = { .port = CAM_SCCB_I2C_PORT, .scl_pin = CAM_SCCB_SCL_PIN, .sda_pin = CAM_SCCB_SDA_PIN },
        .freq = CAM_SCCB_FREQ_HZ,
    },
    /* The Pi 15-pin CSI connector routes none of these to the host. */
    .reset_pin  = -1,
    .pwdn_pin   = -1,
    .signal_pin = -1,
}};

static const esp_video_init_config_t cam_config = {
    .csi = csi_config,
    .cam_motor = motor_config,
};

/* ---- Ring: writing ------------------------------------------------------- */

static inline ring_frame_t *desc_of(uint32_t seq)
{
    return &s_desc[seq % RING_FRAMES];
}

/* Both rings have to still hold the frame: the descriptor slot must not have
 * been reused, and the bytes must not have been written over. */
static inline bool frame_live(uint32_t seq)
{
    if (seq >= s_wseq || (uint32_t)(s_wseq - seq) > RING_FRAMES) {
        return false;
    }
    return (s_wpos - desc_of(seq)->start) <= RING_BYTES;
}

/* Newest frame a decoder could start on, or false while no IDR has been
 * published - which at boot is the first second. Caller holds the lock. */
static bool newest_idr_locked(uint32_t *seq)
{
    for (uint32_t s = s_wseq; s-- > 0 && (s_wseq - s) <= RING_FRAMES; ) {
        if (frame_live(s) && desc_of(s)->is_idr) {
            *seq = s;
            return true;
        }
    }
    return false;
}

static void ring_publish(const uint8_t *data, size_t len, uint32_t pts_ms, bool is_idr)
{
    if (len == 0 || len > RING_BYTES) {
        return;                     /* cannot be stored, let alone streamed */
    }
    if (xSemaphoreTake(s_ring_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return;
    }

    /*
     * Keep every frame contiguous by skipping the tail of the ring when one
     * would straddle the end. A split frame would mean two memcpys everywhere a
     * frame is read, for the sake of at most one frame of padding across a
     * whole megabyte - and the padding counts into s_wpos, so it ages frames
     * out exactly as real bytes would.
     */
    size_t off = (size_t)(s_wpos % RING_BYTES);
    if (off + len > RING_BYTES) {
        s_wpos += RING_BYTES - off;
        off = 0;
    }

    memcpy(s_ring + off, data, len);
    *desc_of(s_wseq) = (ring_frame_t){
        .start = s_wpos, .offset = (uint32_t)off, .len = (uint32_t)len,
        .pts_ms = pts_ms, .is_idr = is_idr,
    };
    s_wpos += len;
    s_wseq++;

    xSemaphoreGive(s_ring_lock);
}

/* ---- Ring: reading ------------------------------------------------------- */

static bool ring_join(uint32_t *seq)
{
    bool found = false;
    if (xSemaphoreTake(s_ring_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }
    found = newest_idr_locked(seq);
    xSemaphoreGive(s_ring_lock);
    return found;
}

/*
 * Copy out frame *seq, resynchronising if it has already been overwritten.
 *
 * A reader that has fallen behind is neither served stale frames nor stalled -
 * it is moved to the newest IDR, which is the only place a decoder can pick the
 * stream back up. The caller sees the jump in *resynced; the timestamps carry
 * the gap, so the picture skips rather than the clock lying.
 *
 * @return bytes copied, or 0 if there is nothing new yet. *seq always advances
 *         past a frame that was delivered or deliberately skipped, so a caller
 *         that loops on 0 cannot livelock.
 */
static size_t ring_read(uint32_t *seq, uint8_t *buf, size_t cap,
                        uint32_t *pts_ms, bool *is_idr, bool *resynced)
{
    size_t n = 0;
    *resynced = false;

    if (xSemaphoreTake(s_ring_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return 0;
    }

    uint32_t want = *seq;
    if (want >= s_wseq) {
        goto out;                   /* nothing new */
    }
    if (!frame_live(want)) {
        uint32_t j;
        if (newest_idr_locked(&j)) {
            want = j;
            *resynced = true;
            s_stat.resyncs++;
        } else {
            *seq = s_wseq;          /* nothing usable left; wait for the next IDR */
            goto out;
        }
    }

    const ring_frame_t *d = desc_of(want);
    *seq = want + 1;
    if (d->len > cap) {
        /* Skipped, not stalled: leaving *seq where it was would make the caller
         * ask for the same impossible frame forever. */
        s_stat.oversize++;
        goto out;
    }
    memcpy(buf, s_ring + d->offset, d->len);
    n = d->len;
    *pts_ms = d->pts_ms;
    *is_idr = d->is_idr;

out:
    xSemaphoreGive(s_ring_lock);
    return n;
}

/* ---- Capture and encode -------------------------------------------------- */

struct encode_ctx {
    int fd;
    uint8_t **buffer;
    size_t frame_bytes;
    esp_h264_enc_handle_t enc;
    uint8_t *enc_out;
    uint32_t enc_out_size;
};

/* Build the fMP4 init segment from the first IDR the encoder produces, because
 * that is the only place the SPS and PPS appear. Taking them from the bitstream
 * rather than constructing them means the container always describes what was
 * actually encoded - including the level rate control settled on, which the
 * configuration does not state. */
static void capture_param_sets(const uint8_t *au, size_t len)
{
    const uint8_t *sps, *pps;
    size_t sps_len, pps_len;

    if (!imx_fmp4_find_param_sets(au, len, &sps, &sps_len, &pps, &pps_len)) {
        ESP_LOGE(TAG, "no SPS/PPS in an IDR frame - /stream.mp4 will not work");
        return;
    }
    imx_fmp4_codec_string(sps, sps_len, s_codec, sizeof(s_codec));
    imx_fmp4_cfg_t cfg = {
        .width = s_w, .height = s_enc_h, .timescale = 1000,
        .sps = sps, .sps_len = sps_len, .pps = pps, .pps_len = pps_len,
    };
    size_t len_out = imx_fmp4_init_segment(&cfg, s_init_seg, sizeof(s_init_seg));
    ESP_LOGI(TAG, "codec %s, SPS %u B, PPS %u B, init segment %u B",
             s_codec, (unsigned)sps_len, (unsigned)pps_len, (unsigned)len_out);
    if (!len_out) {
        ESP_LOGE(TAG, "init segment did not fit IMX_FMP4_INIT_MAX - /stream.mp4 will not work");
        return;
    }
    /* Published last, and only once complete: every reader tests s_init_len to
     * decide whether the stream is describable yet. */
    s_init_len = len_out;
}

/*
 * Encode a few frames with the radio still switched off, and report the mean.
 *
 * The encoder and the WiFi path share this chip, and the only way to know what
 * one costs the other is to measure the same operation with and without. This
 * runs between STREAMON and imx_wifi_connect(), so its number is the encoder
 * alone; /stats reports the same measurement with esp_hosted, LwIP and the
 * servers all running. Comparing the two is a two-line diff in one boot log
 * rather than two firmwares.
 */
static void encoder_warmup(struct encode_ctx *ctx, int frames, const char *label)
{
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uint64_t total = 0, out_total = 0;
    uint32_t worst = 0, done = 0;

    for (int i = 0; i < frames; i++) {
        struct v4l2_buffer buf = { .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(ctx->fd, VIDIOC_DQBUF, &buf) != 0) {
            break;
        }
        esp_h264_enc_in_frame_t in = {
            .raw_data = { .buffer = ctx->buffer[buf.index], .len = ctx->frame_bytes },
            .pts = (uint32_t)i * (1000 / VIDEO_FPS),
        };
        esp_h264_enc_out_frame_t out = {
            .raw_data = { .buffer = ctx->enc_out, .len = ctx->enc_out_size },
        };
        int64_t t = esp_timer_get_time();
        esp_h264_err_t herr = esp_h264_enc_process(ctx->enc, &in, &out);
        uint32_t us = (uint32_t)(esp_timer_get_time() - t);
        ioctl(ctx->fd, VIDIOC_QBUF, &buf);
        if (herr != ESP_H264_ERR_OK) {
            continue;
        }
        total += us;
        out_total += out.length;
        done++;
        if (us > worst) {
            worst = us;
        }
    }
    if (done) {
        ESP_LOGI(TAG, "%s: encode %" PRIu32 " us mean, %" PRIu32 " us worst over %"
                 PRIu32 " frames, %u B/frame (%d us available)",
                 label, (uint32_t)(total / done), worst, done,
                 (unsigned)(out_total / done), 1000000 / VIDEO_FPS);
    }
}

#if ENCODER_BITRATE_SWEEP
/*
 * Encode the same frame over and over, with the camera taken out of the loop.
 *
 * The sweep below showed encode time pinned at ~36.3 ms no matter how many bits
 * came out, which is the signature of a wait rather than of work. The only
 * thing in this system with a ~36 ms period is the camera, so this removes it:
 * one frame is copied into a private buffer, the capture buffer is handed
 * straight back, and the encoder is then run flat out against that copy with
 * nothing to synchronise to. If the time collapses, the encoder was waiting on
 * the pipeline; if it does not, the encoder really is this slow.
 *
 * The first few frames are printed individually - a burst that starts slow and
 * then speeds up is a different story from one that is uniformly slow.
 */
static void encoder_freerun(struct encode_ctx *ctx, int frames, bool camera_off)
{
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uint32_t copy_size = 0;
    uint8_t *copy = esp_h264_aligned_calloc(16, 1, ctx->frame_bytes, &copy_size,
                                            ESP_H264_MEM_SPIRAM);
    if (!copy) {
        ESP_LOGW(TAG, "no PSRAM for the free-run buffer");
        return;
    }

    struct v4l2_buffer buf = { .type = type, .memory = V4L2_MEMORY_MMAP };
    if (ioctl(ctx->fd, VIDIOC_DQBUF, &buf) != 0) {
        free(copy);
        return;
    }
    memcpy(copy, ctx->buffer[buf.index], ctx->frame_bytes);
    ioctl(ctx->fd, VIDIOC_QBUF, &buf);

    if (camera_off) {
        ioctl(ctx->fd, VIDIOC_STREAMOFF, &type);
    }

    uint64_t total = 0;
    uint32_t done = 0;
    for (int i = 0; i < frames; i++) {
        esp_h264_enc_in_frame_t in = {
            .raw_data = { .buffer = copy, .len = ctx->frame_bytes },
            .pts = (uint32_t)i * (1000 / VIDEO_FPS),
        };
        esp_h264_enc_out_frame_t out = {
            .raw_data = { .buffer = ctx->enc_out, .len = ctx->enc_out_size },
        };
        int64_t t = esp_timer_get_time();
        esp_h264_err_t herr = esp_h264_enc_process(ctx->enc, &in, &out);
        uint32_t us = (uint32_t)(esp_timer_get_time() - t);
        if (herr != ESP_H264_ERR_OK) {
            ESP_LOGW(TAG, "free-run frame %d failed (%d)", i, herr);
            continue;
        }
        if (i < 8) {
            ESP_LOGI(TAG, "  free-run frame %d: %" PRIu32 " us, %" PRIu32 " B",
                     i, us, out.length);
        }
        total += us;
        done++;
    }
    if (done) {
        ESP_LOGI(TAG, "free-run, camera %s: encode %" PRIu32 " us mean over %" PRIu32
                 " frames", camera_off ? "stopped" : "streaming",
                 (uint32_t)(total / done), done);
    }

    if (camera_off) {
        for (int i = 0; i < BUFFER_COUNT; i++) {
            struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP, .index = i };
            ioctl(ctx->fd, VIDIOC_QBUF, &b);
        }
        ioctl(ctx->fd, VIDIOC_STREAMON, &type);
    }
    free(copy);
}

/*
 * Sweep the rate-control target and report what each setting costs in time.
 *
 * imx708_video measured 9.2 ms per frame at 4 Mbit/s; this example measured
 * 36 ms at 3 Mbit/s on the same silicon, same resolution, same component
 * version - with the radio off, so the network was not involved. Rather than
 * argue about which difference caused it, this walks the one parameter that
 * changed and prints the curve. The encoder takes a new bitrate between frames,
 * so the whole sweep is one boot and one scene, which is the only way the
 * numbers are comparable at all.
 */
#endif /* ENCODER_BITRATE_SWEEP */

/*
 * Tell rate control how many frames a second it is really being given.
 *
 * Without this the bit budget stays divided by VIDEO_FPS, so at skip 2 every
 * frame is allocated half the bits it should have and the picture is visibly
 * worse for no bandwidth saving - the frames simply arrive half as often.
 */
static void apply_skip(uint8_t skip)
{
    s_skip = skip;
    if (skip == 0) {
        return;                     /* nothing is being encoded to rate-control */
    }
    if (s_enc_param) {
        esp_h264_enc_set_fps((esp_h264_enc_param_handle_t)s_enc_param,
                             (uint8_t)(VIDEO_FPS / skip));
    }
}

#if ENCODER_BITRATE_SWEEP
static void encoder_bitrate_sweep(struct encode_ctx *ctx)
{
    static const uint32_t rates[] = { 1000000, 2000000, 3000000, 4000000, 6000000, 8000000 };
    esp_h264_enc_param_hw_handle_t param = s_enc_param;

    if (!param) {
        ESP_LOGW(TAG, "no parameter handle - skipping the bitrate sweep");
        return;
    }
    for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        if (esp_h264_enc_set_bitrate((esp_h264_enc_param_handle_t)param,
                                     rates[i]) != ESP_H264_ERR_OK) {
            ESP_LOGW(TAG, "set_bitrate(%" PRIu32 ") refused", rates[i]);
            continue;
        }
        char label[32];
        snprintf(label, sizeof(label), "sweep %" PRIu32 " kbit/s", rates[i] / 1000);
        /* Discard a few frames so rate control has settled on the new target
         * before anything is timed. */
        encoder_warmup(ctx, 5, "settling");
        encoder_warmup(ctx, 20, label);
    }
    esp_h264_enc_set_bitrate((esp_h264_enc_param_handle_t)param, VIDEO_BITRATE);
}
#endif /* ENCODER_BITRATE_SWEEP */

/*
 * The whole camera-side pipeline, in one task and one loop: dequeue, encode,
 * requeue, publish. It never touches a socket and never blocks on one, so a
 * slow or absent viewer cannot make the camera drop frames. It is also why the
 * stream keeps running with nobody watching, and so why AE, AWB and autofocus
 * are already converged when someone connects.
 */
static void encode_task(void *arg)
{
    struct encode_ctx *ctx = arg;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int64_t t_first = 0;
    uint8_t skipped_run = 0;

    while (1) {
        struct v4l2_buffer buf = { .type = type, .memory = V4L2_MEMORY_MMAP };
        /*
         * Timed, because it is the one number that says who the bottleneck is.
         * A long wait here means the camera is pacing the loop and the encoder
         * has headroom; a wait near zero means a frame was always already
         * waiting, so the encoder is the limit and frames are being dropped
         * between the two.
         */
        int64_t t_dq = esp_timer_get_time();
        int dq_err = ioctl(ctx->fd, VIDIOC_DQBUF, &buf);
        uint32_t dq_us = (uint32_t)(esp_timer_get_time() - t_dq);
        if (dq_err != 0) {
            ESP_LOGE(TAG, "DQBUF failed after %" PRIu32 " frames - encoder task stopping",
                     s_stat.encoded);
            vTaskDelete(NULL);
            return;
        }

        s_stat.dq_us_total += dq_us;
        if (dq_us > s_stat.dq_us_max) {
            s_stat.dq_us_max = dq_us;
        }

        /*
         * Dropped before the encoder, not after: the point of skipping is to
         * give the chip back, so the frame has to cost nothing but the DQBUF
         * and QBUF that keep the camera's queue turning over.
         */
        if (s_skip == 0 || ++skipped_run < s_skip) {
            ioctl(ctx->fd, VIDIOC_QBUF, &buf);
            s_stat.skipped++;
            continue;
        }
        skipped_run = 0;

        int64_t now = esp_timer_get_time();
        if (t_first == 0) {
            t_first = now;
            s_stat.t_start = now;
        }
        uint32_t pts_ms = (uint32_t)((now - t_first) / 1000);

        esp_h264_enc_in_frame_t in_frame = {
            .raw_data = { .buffer = ctx->buffer[buf.index], .len = ctx->frame_bytes },
            .pts = pts_ms,
        };
        esp_h264_enc_out_frame_t out_frame = {
            .raw_data = { .buffer = ctx->enc_out, .len = ctx->enc_out_size },
        };

        int64_t t_enc = esp_timer_get_time();
        esp_h264_err_t herr = esp_h264_enc_process(ctx->enc, &in_frame, &out_frame);
        uint32_t enc_us = (uint32_t)(esp_timer_get_time() - t_enc);

        /* esp_h264_enc_process is synchronous, so the input DMA has finished by
         * the time it returns; holding the buffer any longer only costs the
         * camera a place to put the next frame. */
        ioctl(ctx->fd, VIDIOC_QBUF, &buf);

        if (herr != ESP_H264_ERR_OK) {
            /* The component forces the next frame back to IDR after a failure,
             * so the stream resynchronises by itself and a viewer sees one
             * skipped frame. */
            s_stat.failed++;
            ESP_LOGW(TAG, "encode failed (%d) on frame %" PRIu32 "%s", herr, s_stat.encoded,
                     herr == ESP_H264_ERR_OVERFLOW ? " - raise ENC_OUT_BYTES" : "");
            continue;
        }

        bool is_idr = (out_frame.frame_type == ESP_H264_FRAME_TYPE_IDR);
        if (!s_init_len && is_idr) {
            capture_param_sets(ctx->enc_out, out_frame.length);
        }

        ring_publish(ctx->enc_out, out_frame.length, pts_ms, is_idr);

        s_stat.encoded++;
        s_stat.idr += is_idr ? 1 : 0;
        s_stat.bytes += out_frame.length;
        s_stat.enc_us_total += enc_us;
        if (enc_us > s_stat.enc_us_max) {
            s_stat.enc_us_max = enc_us;
        }
    }
}

/* ---- Streaming ----------------------------------------------------------- */

/* Give up if the camera has stopped producing, so a dead pipeline releases the
 * server rather than pinning it on a socket that will never be written to. */
#define STREAM_STALL_MS     5000

/*
 * Wait for the next frame this reader has not seen.
 *
 * A 5 ms poll rather than a notification: frames arrive every 36 ms, so this
 * costs at most 5 ms of added latency and a handful of wakeups per frame, and
 * it keeps the writer free of any knowledge of how many readers exist. It also
 * yields, which matters - the watchdog is unforgiving of a send path that never
 * sleeps.
 */
static size_t stream_next(uint32_t *seq, uint8_t *buf, size_t cap,
                          uint32_t *pts_ms, bool *is_idr, bool *resynced)
{
    /*
     * A wall-clock deadline, and a delay of one tick rather than a duration.
     *
     * The obvious form of this loop - count iterations, vTaskDelay(pdMS_TO_TICKS(5))
     * - is a trap, and it cost a long detour here. pdMS_TO_TICKS(5) is
     * (5 * CONFIG_FREERTOS_HZ) / 1000, which at the IDF default of 100 Hz is
     * *zero*: vTaskDelay(0) yields without waiting, so the loop burned its
     * whole five-second budget in ten milliseconds and dropped every viewer
     * roughly as fast as it could accept one. From the far end that looks
     * exactly like a slow network - a few frames arrive, the connection closes,
     * and every throughput figure computed over the intended window is wrong by
     * the ratio of the two.
     *
     * esp_timer_get_time() does not care what the tick rate is, and
     * vTaskDelay(1) is always the shortest real sleep there is.
     */
    int64_t deadline = esp_timer_get_time() + (int64_t)STREAM_STALL_MS * 1000;

    while (esp_timer_get_time() < deadline) {
        size_t n = ring_read(seq, buf, cap, pts_ms, is_idr, resynced);
        if (n) {
            return n;
        }
        vTaskDelay(1);
    }
    return 0;
}

/* How long the socket took to accept one frame. Near the frame interval means
 * the link is the bottleneck; near zero means the pipeline is. */
static void note_send(int64_t us)
{
    s_stat.send_us_total += (uint64_t)us;
    s_stat.send_frames++;
    if ((uint32_t)us > s_stat.send_us_max) {
        s_stat.send_us_max = (uint32_t)us;
    }
}

/*
 * "not yet", as opposed to "no".
 *
 * The page polls both stream endpoints from a cold boot, and for the first
 * second there is genuinely nothing to send - the encoder has not produced a
 * keyframe. httpd_resp_send_err() is the wrong shape for that: it takes an
 * enum that IDF v5.4 has no 503 in, and it purges the socket, so a client that
 * would have retried on the same connection has to reconnect instead.
 */
static esp_err_t send_503(httpd_req_t *req, const char *why)
{
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Retry-After", "1");
    httpd_resp_send(req, why, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/*
 * Set up a streaming response, and turn Nagle off on its socket.
 *
 * TCP_NODELAY is the single most important line in this file. Nagle's algorithm
 * holds a sub-MSS write until the previous one is acknowledged, which is right
 * for a chatty protocol and wrong for this: a frame is written, then the sender
 * goes quiet for 36 ms, so the tail of every frame waits on an ACK the receiver
 * is in no hurry to send - it has nothing to say and delays it. The two
 * behaviours interlock and the stream advances roughly once per delayed-ACK
 * timer instead of once per frame.
 *
 * Measured here before the fix: send_us_mean 40 902 us per frame - 41 ms blocked
 * in a socket for ~9.5 KB - with 2 to 5 fps arriving at the client while the
 * encoder produced 27. /bench never showed it, because /bench always has more
 * data queued so its writes are never sub-MSS and Nagle never engages; that is
 * exactly why a bulk benchmark cannot stand in for a paced one.
 *
 * The Access-Control-Allow-Origin header is not needed by this example's own
 * page - which is served from this same origin, on purpose - but it costs
 * nothing and lets the stream be pulled into a page served from anywhere else.
 */
static void stream_headers(httpd_req_t *req, const char *type)
{
    int fd = httpd_req_to_sockfd(req);
    int one = 1;
    if (fd >= 0 && setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) != 0) {
        ESP_LOGW(TAG, "TCP_NODELAY refused - expect a few frames a second");
    }
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
}

/*
 * One write per frame, chunk framing included.
 *
 * httpd_resp_send_chunk() sends the size line, the body and the trailing CRLF
 * as three separate calls. That is invisible with Nagle on, because the kernel
 * coalesces them - but Nagle is off here for the reasons above, so it becomes
 * three packets a frame, two of them a handful of bytes. On a 2.4 GHz link a
 * packet costs airtime and an ACK almost regardless of its size, and at 27 fps
 * that is 54 near-empty transmissions a second competing with the payload.
 *
 * The caller leaves CHUNK_GAP bytes free in front of the payload and two behind
 * it; the size line is written backwards into that gap so the whole chunk goes
 * out as one buffer. httpd_send() is allowed to send less than asked, so the
 * short write has to be looped over - httpd_resp_send_chunk() was doing that
 * part for us.
 */
#define CHUNK_GAP   12

static esp_err_t stream_send_framed(httpd_req_t *req, uint8_t *scratch, size_t len)
{
    char hdr[CHUNK_GAP];
    int hl = snprintf(hdr, sizeof(hdr), "%X\r\n", (unsigned)len);
    if (hl <= 0 || hl > CHUNK_GAP) {
        return ESP_FAIL;
    }
    uint8_t *start = scratch + CHUNK_GAP - hl;
    memcpy(start, hdr, hl);
    memcpy(scratch + CHUNK_GAP + len, "\r\n", 2);

    size_t total = (size_t)hl + len + 2;
    size_t sent = 0;
    while (sent < total) {
        int n = httpd_send(req, (const char *)start + sent, total - sent);
        if (n <= 0) {
            return ESP_FAIL;        /* the viewer is gone */
        }
        sent += (size_t)n;
    }
    return ESP_OK;
}

static void viewer_joined(const char *what, uint32_t seq)
{
    s_stat.viewers++;
    ESP_LOGI(TAG, "%s viewer joined at frame %" PRIu32 " (%" PRId32 " watching)",
             what, seq, s_stat.viewers);
}

static void viewer_left(const char *what, uint32_t frames, uint64_t bytes, int64_t t0)
{
    s_stat.viewers--;
    uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    uint32_t kbit_s = ms ? (uint32_t)(bytes * 8 / ms) : 0;
    ESP_LOGI(TAG, "%s viewer left after %" PRIu32 " frames, %u KB in %" PRIu32 " ms"
             " -> %" PRIu32 ".%" PRIu32 " Mbit/s (%" PRId32 " watching)",
             what, frames, (unsigned)(bytes / 1024), ms,
             kbit_s / 1000, (kbit_s / 100) % 10, s_stat.viewers);
}

/*
 * Live fragmented MP4: the init segment, then one moof+mdat per frame.
 *
 * Each frame is held back until the next one arrives, so its fragment can carry
 * the real interval as its duration rather than a nominal one. It costs a frame
 * of latency - 36 ms against a pipeline already hundreds of milliseconds deep -
 * and buys a timeline where each fragment's decode time is exactly the sum of
 * the durations before it. MSE is unforgiving of the alternative: a nominal
 * duration against a true timestamp leaves a few milliseconds of gap or overlap
 * on every frame, and the player either stalls in the gaps or drifts out of
 * them.
 */
static esp_err_t stream_mp4_handler(httpd_req_t *req)
{
    if (!s_init_len) {
        return send_503(req, "no keyframe encoded yet - try again in a second");
    }

    /*
     * Three buffers per viewer, each ENC_OUT_BYTES: the frame just read, the
     * frame held back for its duration, and the fragment built from it. 1.5 MB
     * of PSRAM for a stream whose frames average about 13 KB, which is the
     * price of sizing for the worst IDR the encoder is allowed to emit rather
     * than for the ones it does.
     */
    uint8_t *a = heap_caps_malloc(ENC_OUT_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *b = heap_caps_malloc(ENC_OUT_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    /* The fragment is built CHUNK_GAP bytes in, leaving room for the chunk size
     * line in front of it and the CRLF behind, so it can go out as one write. */
    uint8_t *frag = heap_caps_malloc(CHUNK_GAP + ENC_OUT_BYTES + IMX_FMP4_FRAG_OVERHEAD + 2,
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t seq;

    if (!a || !b || !frag) {
        free(a);
        free(b);
        free(frag);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no PSRAM for a viewer");
        return ESP_FAIL;
    }
    if (!ring_join(&seq)) {
        free(a);
        free(b);
        free(frag);
        return send_503(req, "no keyframe in the ring yet");
    }

    viewer_joined("mp4", seq);
    stream_headers(req, "video/mp4");

    esp_err_t ret = httpd_resp_send_chunk(req, (const char *)s_init_seg, s_init_len);

    uint8_t *cur = a, *held = b;
    size_t held_len = 0;
    uint32_t held_pts = 0;
    uint32_t frag_seq = 1, frames = 0;
    uint64_t bytes = s_init_len;
    int64_t t0 = esp_timer_get_time();

    while (ret == ESP_OK) {
        uint32_t pts;
        bool is_idr, resynced;
        size_t n = stream_next(&seq, cur, ENC_OUT_BYTES, &pts, &is_idr, &resynced);
        if (!n) {
            ESP_LOGW(TAG, "no frames for %d ms - dropping the mp4 viewer", STREAM_STALL_MS);
            break;
        }

        if (held_len) {
            uint32_t dur = pts - held_pts;
            /* A resync leaves a real gap and the timestamps should show it, but
             * an implausible one is a wrapped or bogus clock, not a pause. */
            if (dur == 0 || dur > 2000) {
                dur = 1000 / VIDEO_FPS;
            }
            size_t flen = imx_fmp4_fragment(held, held_len, frag_seq, held_pts, dur,
                                            frag + CHUNK_GAP,
                                            ENC_OUT_BYTES + IMX_FMP4_FRAG_OVERHEAD);
            if (flen) {
                int64_t t_send = esp_timer_get_time();
                ret = stream_send_framed(req, frag, flen);
                note_send(esp_timer_get_time() - t_send);
                bytes += flen;
                s_stat.sent += flen;
                frag_seq++;
                frames++;
            } else {
                /* An access unit with no coded slice in it, or one that would
                 * not fit. Dropping it is right either way - it is not a
                 * picture. */
                ESP_LOGW(TAG, "frame %" PRIu32 " (%u B) would not mux", seq - 1, (unsigned)held_len);
            }
        }

        /* The frame just read becomes the held one; the buffer it displaces is
         * where the next read lands. A swap rather than a copy, so nothing is
         * moved twice. */
        uint8_t *t = held;
        held = cur;
        cur = t;
        held_len = n;
        held_pts = pts;
    }

    if (ret == ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
    }
    free(a);
    free(b);
    free(frag);
    viewer_left("mp4", frames, bytes, t0);
    return ret;
}

/*
 * The same frames as Annex-B, straight off the encoder.
 *
 * No container, no muxing, nothing this file could get wrong - which is the
 * point: when the browser shows nothing, this endpoint answers whether the
 * problem is the camera, the encoder and the link, or only the MP4 boxes. It is
 * also what ffplay, VLC and ffmpeg take directly, so recording the stream needs
 * no tooling on this side.
 */
static esp_err_t stream_h264_handler(httpd_req_t *req)
{
    /* Frames land CHUNK_GAP bytes in, so the chunk framing can be written
     * around them in place and the whole thing sent as one write. */
    uint8_t *buf = heap_caps_malloc(CHUNK_GAP + ENC_OUT_BYTES + 2,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t seq;

    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no PSRAM for a viewer");
        return ESP_FAIL;
    }
    if (!ring_join(&seq)) {
        free(buf);
        return send_503(req, "no keyframe in the ring yet");
    }

    viewer_joined("h264", seq);
    stream_headers(req, "video/H264");

    esp_err_t ret = ESP_OK;
    uint32_t frames = 0;
    uint64_t bytes = 0;
    int64_t t0 = esp_timer_get_time();

    while (ret == ESP_OK) {
        uint32_t pts;
        bool is_idr, resynced;
        size_t n = stream_next(&seq, buf + CHUNK_GAP, ENC_OUT_BYTES, &pts, &is_idr, &resynced);
        if (!n) {
            ESP_LOGW(TAG, "no frames for %d ms - dropping the h264 viewer", STREAM_STALL_MS);
            break;
        }
        int64_t t_send = esp_timer_get_time();
        /*
         * The first payload goes through httpd, which is what writes the status
         * line and headers; only after that can the raw single-write path be
         * used, or the response would have no head.
         */
        if (frames == 0) {
            ret = httpd_resp_send_chunk(req, (const char *)buf + CHUNK_GAP, n);
        } else {
            ret = stream_send_framed(req, buf, n);
        }
        note_send(esp_timer_get_time() - t_send);
        bytes += n;
        s_stat.sent += n;
        frames++;
    }

    if (ret == ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
    }
    free(buf);
    viewer_left("h264", frames, bytes, t0);
    return ret;
}

/* ---- Control endpoints --------------------------------------------------- */

/*
 * Network only: one PSRAM block sent over and over, no camera and no encoder in
 * the loop. Carried over unchanged from imx708_wifi_snapshot so the two
 * firmwares can be compared directly - which is the whole point of it, because
 * the interesting question here is not "how fast is the radio" but "how fast is
 * the radio *while the encoder is running*". It lives on the control server, so
 * it can be run against a stream that is already playing.
 */
#define BENCH_BYTES         (8 * 1024 * 1024)
#define BENCH_CHUNK         (16 * 1024)

static esp_err_t bench_handler(httpd_req_t *req)
{
    uint8_t *block = heap_caps_malloc(BENCH_CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!block) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no PSRAM");
        return ESP_FAIL;
    }
    /* A counter, not zeros: a compressing middlebox or a driver that elides
     * runs of zeros would otherwise flatter the result. */
    for (size_t i = 0; i < BENCH_CHUNK; i++) {
        block[i] = (uint8_t)(i * 31u + (i >> 8));
    }

    httpd_resp_set_type(req, "application/octet-stream");

    int64_t t0 = esp_timer_get_time();
    size_t sent = 0;
    esp_err_t ret = ESP_OK;
    while (sent < BENCH_BYTES) {
        size_t n = BENCH_BYTES - sent;
        if (n > BENCH_CHUNK) {
            n = BENCH_CHUNK;
        }
        ret = httpd_resp_send_chunk(req, (const char *)block, n);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "bench aborted after %u bytes", (unsigned)sent);
            free(block);
            return ret;
        }
        sent += n;
    }
    httpd_resp_send_chunk(req, NULL, 0);

    uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    uint32_t kbit_s = ms ? (uint32_t)((uint64_t)BENCH_BYTES * 8 / ms) : 0;
    ESP_LOGI(TAG, "bench: %d bytes in %" PRIu32 " ms -> %" PRIu32 ".%" PRIu32 " Mbit/s",
             BENCH_BYTES, ms, kbit_s / 1000, (kbit_s / 100) % 10);
    free(block);
    return ESP_OK;
}

/*
 * GET /skip?n=<1..8> - change the frame rate while the stream is running.
 *
 * A knob rather than a constant because the interesting number is not the frame
 * rate but what the frame rate costs: this is what makes it possible to read
 * /bench and the delivered frame rate at each setting in one boot, on one
 * scene, over one WiFi association. Two of those three move enough between
 * reboots to swamp the effect being measured.
 */
static esp_err_t skip_handler(httpd_req_t *req)
{
    char query[32], val[8];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK
            && httpd_query_key_value(query, "n", val, sizeof(val)) == ESP_OK) {
        int n = atoi(val);
        if (n < 0 || n > 8) {
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_send(req, "n must be 0..8, 0 stops the encoder" "\n", HTTPD_RESP_USE_STRLEN);
        }
        apply_skip((uint8_t)n);
        ESP_LOGI(TAG, "frame skip set to %d - encoding %d of every %d frames",
                 n, 1, n);
    }
    char body[64];
    int n = snprintf(body, sizeof(body), "skip=%u fps=%u" "\n",
                     s_skip, s_skip ? VIDEO_FPS / s_skip : 0);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}

/*
 * GET /rate?kbit=<250..8000> - retarget rate control while the stream runs.
 *
 * This, not /skip, is the bandwidth knob: rate control's budget is bits per
 * second, so this is the only setting that changes how much of the link the
 * stream asks for. It matters because the link is not a constant - /bench on
 * the *snapshot* firmware measured 7.2 Mbit/s one day and 0.05 Mbit/s the next,
 * same board, same room, same -40 dBm - so the right bitrate is a thing you
 * find by looking, not a thing you can compile in.
 *
 * Takes effect on the next frame; no reconnect, and the stream does not break,
 * because bitrate is not part of what the init segment declared.
 */
static esp_err_t rate_handler(httpd_req_t *req)
{
    char query[32], val[12];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK
            && httpd_query_key_value(query, "kbit", val, sizeof(val)) == ESP_OK) {
        int kbit = atoi(val);
        if (kbit < 250 || kbit > 8000) {
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_send(req, "kbit must be 250..8000\n", HTTPD_RESP_USE_STRLEN);
        }
        if (!s_enc_param) {
            return send_503(req, "no encoder parameter handle\n");
        }
        esp_h264_enc_set_bitrate((esp_h264_enc_param_handle_t)s_enc_param,
                                 (uint32_t)kbit * 1000);
        s_bitrate = (uint32_t)kbit * 1000;
        ESP_LOGI(TAG, "rate control retargeted to %d kbit/s", kbit);
    }
    char body[48];
    int n = snprintf(body, sizeof(body), "kbit=%" PRIu32 "\n", s_bitrate / 1000);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}

static esp_err_t info_handler(httpd_req_t *req)
{
    char body[256];
    int n = snprintf(body, sizeof(body),
                     "{\"ready\":%s,\"width\":%" PRIu32 ",\"height\":%" PRIu32
                     ",\"fps\":%d,\"skip\":%u,\"gop\":%d,\"bitrate\":%" PRIu32 ",\"codec\":\"%s\""
                     ",\"stream_port\":%d}\n",
                     s_init_len ? "true" : "false", s_w, s_enc_h,
                     s_skip ? VIDEO_FPS / s_skip : 0, s_skip, VIDEO_GOP, s_bitrate,
                     s_codec, STREAM_PORT);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}

static esp_err_t stats_handler(httpd_req_t *req)
{
    /*
     * fps_recent covers only the interval since the previous /stats. The
     * cumulative figure cannot show the effect of anything changed while the
     * board was running - it stays dominated by however long the old setting
     * had already been averaging.
     */
    static int64_t prev_us;
    static uint32_t prev_frames;
    int64_t now_us = esp_timer_get_time();
    uint32_t window_ms = prev_us ? (uint32_t)((now_us - prev_us) / 1000) : 0;
    uint32_t recent_x10 = window_ms
        ? (uint32_t)((uint64_t)(s_stat.encoded - prev_frames) * 10000 / window_ms) : 0;
    prev_us = now_us;
    prev_frames = s_stat.encoded;

    uint32_t ms = s_stat.t_start
        ? (uint32_t)((esp_timer_get_time() - s_stat.t_start) / 1000) : 0;
    uint32_t fps_x10 = ms ? (uint32_t)((uint64_t)s_stat.encoded * 10000 / ms) : 0;
    uint32_t kbit_s = ms ? (uint32_t)(s_stat.bytes * 8 / ms) : 0;
    uint32_t mean_us = s_stat.encoded ? (uint32_t)(s_stat.enc_us_total / s_stat.encoded) : 0;
    uint32_t dq_mean = s_stat.encoded ? (uint32_t)(s_stat.dq_us_total / s_stat.encoded) : 0;
    uint32_t send_mean = s_stat.send_frames
        ? (uint32_t)(s_stat.send_us_total / s_stat.send_frames) : 0;

    char body[768];
    int n = snprintf(body, sizeof(body),
                     "uptime_ms=%" PRIu32 "\n"
                     "frames_encoded=%" PRIu32 "\nidr=%" PRIu32 "\nencode_failed=%" PRIu32 "\n"
                     "fps=%" PRIu32 ".%" PRIu32 "\n"
                     "fps_recent=%" PRIu32 ".%" PRIu32 "\n"
                     "encode_us_mean=%" PRIu32 "\nencode_us_worst=%" PRIu32
                     "\nencode_us_budget=%d\n"
                     "dqbuf_us_mean=%" PRIu32 "\ndqbuf_us_worst=%" PRIu32 "\n"
                     "send_us_mean=%" PRIu32 "\nsend_us_worst=%" PRIu32
                     "\nframes_sent=%" PRIu32 "\n"
                     "encoded_kbit_s=%" PRIu32 "\ntarget_kbit_s=%" PRIu32 "\n"
                     "viewers=%" PRId32 "\nresyncs=%" PRIu32 "\noversize_skipped=%" PRIu32 "\n"
                     "frame_skip=%u\nframes_skipped=%" PRIu32 "\n"
                     "bytes_sent=%llu\n"
                     "heap_free=%u\npsram_free=%u\n",
                     ms, s_stat.encoded, s_stat.idr, s_stat.failed,
                     fps_x10 / 10, fps_x10 % 10,
                     recent_x10 / 10, recent_x10 % 10,
                     mean_us, s_stat.enc_us_max, 1000000 / VIDEO_FPS,
                     dq_mean, s_stat.dq_us_max,
                     send_mean, s_stat.send_us_max, s_stat.send_frames,
                     kbit_s, s_bitrate / 1000,
                     s_stat.viewers, s_stat.resyncs, s_stat.oversize,
                     s_skip, s_stat.skipped,
                     (unsigned long long)s_stat.sent,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}

/*
 * The player.
 *
 * A <video> element cannot be pointed at a live stream directly - src wants
 * something seekable with a known length - so the page reads the fragmented MP4
 * with fetch() and feeds the bytes to Media Source Extensions itself. The
 * chunks fetch() hands back are arbitrary byte slices with no relation to
 * fragment boundaries, which is fine: a SourceBuffer parses incrementally and
 * only needs the bytes in order.
 *
 * The two housekeeping jobs are what keep it live rather than merely playing.
 * Buffered media is dropped once it is well behind the playhead, or the browser
 * accumulates the whole session in memory; and the playhead is pushed to the
 * live edge whenever it drifts more than a couple of seconds back, which is
 * what happens after any stall.
 */
/* http://<ip>/ is what anyone will try first, and it is the wrong port for
 * watching. Send them to the right one rather than explaining. */
static esp_err_t redirect_handler(httpd_req_t *req)
{
    /* Location cannot carry a different port as a relative URL, and the board
     * does not know which of its addresses the client used - so build it from
     * the request's own Host header, with any port it carried stripped off. */
    char host[40] = "";
    if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK) {
        return send_503(req, "no Host header to redirect against");
    }
    char *colon = strchr(host, ':');
    if (colon) {
        *colon = '\0';
    }
    char to[64];
    snprintf(to, sizeof(to), "http://%s:%d/", host, STREAM_PORT);
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", to);
    return httpd_resp_send(req, to, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t index_handler(httpd_req_t *req)
{
    static const char page[] =
"<!doctype html><meta name=viewport content=\"width=device-width,initial-scale=1\">"
"<title>IMX708 live</title>"
"<style>body{margin:0;background:#111;color:#eee;font:14px system-ui;text-align:center}"
"video{max-width:100%;height:auto;background:#000}#s{padding:8px;color:#9ab;font:12px ui-monospace,monospace}"
"a{color:#6cf}</style>"
"<video id=v autoplay muted playsinline></video><div id=s>starting...</div>"
"<div id=lk></div>"
"<script>\n"
"var v=document.getElementById('v'),S=document.getElementById('s');\n"
"var ms,sb,q=[],info,mime,bytes=0,t0=0;\n"
"var CTRL=" IMX_STR(CTRL_PORT) ";\n"
"function say(t){S.textContent=t}\n"
"function edge(){return sb&&sb.buffered.length?sb.buffered.end(sb.buffered.length-1):0}\n"
"function lag(){return Math.max(0,edge()-v.currentTime)}\n"
/* Returns true when it started an operation, so the caller does not then try to
   append into a SourceBuffer that is already updating. */
"function tidy(){if(!sb||sb.updating||!sb.buffered.length)return false;\n"
"  var keep=edge()-10;\n"
"  if(sb.buffered.start(0)<keep-1){sb.remove(0,keep);return true}\n"
"  if(lag()>2){v.currentTime=edge()-0.2}\n"
"  return false}\n"
"function pump(){if(sb&&!sb.updating&&q.length)sb.appendBuffer(q.shift())}\n"
"async function open_(){\n"
"  sb=ms.addSourceBuffer(mime); sb.mode='segments';\n"
"  sb.addEventListener('updateend',function(){if(!tidy())pump()});\n"
/* Same origin as the page, which is why the page is served by the stream
   server too: no CORS, no second origin for a client to object to. */
"  var r=(await fetch('/stream.mp4')).body.getReader(); t0=performance.now();\n"
"  for(;;){\n"
"    var c=await r.read();\n"
"    if(c.done){say('stream ended');break}\n"
"    q.push(c.value); bytes+=c.value.length; pump();\n"
"    if(v.paused)v.play().catch(function(){});\n"
"    var dt=(performance.now()-t0)/1000;\n"
"    if(dt>1)say((bytes*8/dt/1e6).toFixed(2)+' Mbit/s   '+lag().toFixed(1)+' s behind live   '\n"
"      +(bytes/1024|0)+' KB   '+info.width+'x'+info.height+' '+info.codec);\n"
"  }\n"
"}\n"
"async function start(){\n"
"  for(;;){ info=await (await fetch('/info')).json();\n"
"    if(info.ready)break;\n"
"    say('waiting for the first keyframe...'); await new Promise(function(r){setTimeout(r,500)}) }\n"
"  mime='video/mp4; codecs=\"'+info.codec+'\"';\n"
"  if(!window.MediaSource||!MediaSource.isTypeSupported(mime)){\n"
"    say('this browser will not play '+mime+' - try: ffplay '+location.origin\n"
"      +'/stream.h264'); return }\n"
"  ms=new MediaSource(); v.src=URL.createObjectURL(ms);\n"
"  ms.addEventListener('sourceopen',open_,{once:true});\n"
"}\n"
/* The control endpoints live on the other server so they stay answerable while
   a stream is running. Plain links, not fetches: a navigation is not subject to
   the cross-origin rules a fetch is. */
"document.getElementById('lk').innerHTML=['stats','bench','info'].map(function(k){\n"
"  return \"<a href='http://\"+location.hostname+':'+CTRL+'/'+k+\"'>\"+k+'</a>'}).join(' &middot; ');\n"
"start().catch(function(e){say('failed: '+e)});\n"
"</script>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, sizeof(page) - 1);
}

/* ---- Servers ------------------------------------------------------------- */

static httpd_handle_t start_httpd(uint16_t port, uint16_t ctrl_port, size_t stack,
                                  uint16_t sockets, const httpd_uri_t *uris, size_t n_uris)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = port;
    /*
     * Two instances cannot share a control port. httpd uses this UDP socket to
     * wake its own task, it defaults to 32768 for every instance, and the
     * second httpd_start() fails on the bind with nothing to say about why.
     */
    cfg.ctrl_port = ctrl_port;
    cfg.stack_size = stack;
    cfg.max_open_sockets = sockets;
    cfg.max_uri_handlers = (uint16_t)n_uris;
    cfg.lru_purge_enable = true;
    /*
     * A stream handler blocks in send while a viewer's window is full, which on
     * this link happens routinely. The snapshot example's 10 s would drop a
     * viewer mid-hiccup; the stream is written frame by frame, so a stall long
     * enough to matter is caught by STREAM_STALL_MS instead.
     */
    cfg.send_wait_timeout = 20;
    cfg.recv_wait_timeout = 20;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed on port %u", port);
        return NULL;
    }
    for (size_t i = 0; i < n_uris; i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
    return server;
}

static bool start_servers(void)
{
    /* The instruments. Nothing here blocks, so it answers during a stream. */
    static const httpd_uri_t ctrl_uris[] = {
        { .uri = "/stats", .method = HTTP_GET, .handler = stats_handler },
        { .uri = "/bench", .method = HTTP_GET, .handler = bench_handler },
        { .uri = "/skip",  .method = HTTP_GET, .handler = skip_handler },
        { .uri = "/rate",  .method = HTTP_GET, .handler = rate_handler },
        { .uri = "/info",  .method = HTTP_GET, .handler = info_handler },
        /* The page is here too, so http://<ip>/ is not a dead end - but it
         * redirects, because a page served from this origin could not fetch the
         * stream without the cross-origin permission the note above is about. */
        { .uri = "/",      .method = HTTP_GET, .handler = redirect_handler },
    };
    /* The viewer. One origin: page, metadata and media. */
    static const httpd_uri_t stream_uris[] = {
        { .uri = "/",            .method = HTTP_GET, .handler = index_handler },
        { .uri = "/info",        .method = HTTP_GET, .handler = info_handler },
        { .uri = "/stream.mp4",  .method = HTTP_GET, .handler = stream_mp4_handler },
        { .uri = "/stream.h264", .method = HTTP_GET, .handler = stream_h264_handler },
    };

    if (!start_httpd(CTRL_PORT, 32768, 6144, 4,
                     ctrl_uris, sizeof(ctrl_uris) / sizeof(ctrl_uris[0]))) {
        return false;
    }
    /*
     * Three sockets on the stream server: the viewer's stream, the page and
     * /info requests that precede it, and one spare so a reload is accepted
     * before lru_purge_enable reclaims the socket the old viewer left.
     */
    if (!start_httpd(STREAM_PORT, 32769, 8192, 3,
                     stream_uris, sizeof(stream_uris) / sizeof(stream_uris[0]))) {
        return false;
    }
    return true;
}

/* ---- Bring-up ------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "IMX708 live H.264 over WiFi");

#if AF_DEBUG_LOG
    esp_log_level_set("esp_ipa_af", ESP_LOG_DEBUG);
#endif

    /* Credentials before the camera: discovering they are missing after the
     * sensor is streaming wastes the run and buries the message under ISP
     * logs. */
    if (!imx_wifi_have_credentials()) {
        (void)imx_wifi_connect(NULL, 0);   /* prints what to write */
        printf("\n==== done - no credentials ====\n");
        return;
    }

    if (esp_video_init(&cam_config) != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed");
        printf("\n==== done - camera init failed ====\n");
        return;
    }

    int fd = open(CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed", CAM_DEV_PATH);
        printf("\n==== done - camera open failed ====\n");
        return;
    }

    /*
     * YUV420 out of the ISP, not the snapshot example's RGB565. The P4's H.264
     * core does not take RGB: it wants YUV420 in the packed layout Espressif
     * call O_UYY_E_VYY, and the ISP's YUV420 output *is* that layout, because
     * the two blocks were designed to hand off to each other. So there is no
     * colour conversion anywhere in this example - and it drops the frame from
     * 4.1 MB to 3.1 MB on the way.
     */
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_format fmt = { .type = type };
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "G_FMT failed");
        printf("\n==== done - camera format failed ====\n");
        return;
    }
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0 || ioctl(fd, VIDIOC_G_FMT, &fmt) != 0
            || fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) {
        ESP_LOGE(TAG, "the ISP would not give us YUV420 - the H.264 core takes nothing else");
        printf("\n==== done - camera format failed ====\n");
        return;
    }

    uint32_t w = fmt.fmt.pix.width, h = fmt.fmt.pix.height;
#if ENCODE_16_ALIGNED
    uint32_t enc_h = h & ~15u;
#else
    uint32_t enc_h = h;
#endif
    s_w = w;
    s_enc_h = enc_h;
    size_t frame_bytes = (size_t)w * enc_h * 3 / 2;
    ESP_LOGI(TAG, "format %" PRIu32 "x%" PRIu32 ", encoding %" PRIu32 "x%" PRIu32
             " (%u bytes/frame)", w, h, w, enc_h, (unsigned)frame_bytes);

    static uint8_t *buffer[BUFFER_COUNT];
    struct v4l2_requestbuffers req = { .count = BUFFER_COUNT, .type = type, .memory = V4L2_MEMORY_MMAP };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "REQBUFS failed - not enough PSRAM for %d x %" PRIu32 " byte frames?",
                 BUFFER_COUNT, fmt.fmt.pix.sizeimage);
        printf("\n==== done - out of memory ====\n");
        return;
    }
    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP, .index = i };
        ioctl(fd, VIDIOC_QUERYBUF, &b);
        buffer[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        ioctl(fd, VIDIOC_QBUF, &b);
    }

    s_ring = heap_caps_malloc(RING_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_ring_lock = xSemaphoreCreateMutex();
    /*
     * The encoder's output buffer is written by DMA and then invalidated
     * wholesale by the component, so it has to be cache-line aligned in both
     * address and length - an ordinary malloc is not.
     */
    uint32_t enc_out_size = 0;
    uint8_t *enc_out = esp_h264_aligned_calloc(16, 1, ENC_OUT_BYTES, &enc_out_size,
                                               ESP_H264_MEM_SPIRAM);
    if (!s_ring || !s_ring_lock || !enc_out) {
        ESP_LOGE(TAG, "no PSRAM for the frame ring or the encoder output");
        printf("\n==== done - out of memory ====\n");
        return;
    }

    esp_h264_enc_cfg_hw_t enc_cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop      = VIDEO_GOP,
        .fps      = VIDEO_FPS,
        .res      = { .width = (uint16_t)w, .height = (uint16_t)enc_h },
        .rc       = { .bitrate = VIDEO_BITRATE, .qp_min = VIDEO_QP_MIN, .qp_max = VIDEO_QP_MAX },
    };
    esp_h264_enc_handle_t enc = NULL;
    if (esp_h264_enc_hw_new(&enc_cfg, &enc) != ESP_H264_ERR_OK
            || esp_h264_enc_open(enc) != ESP_H264_ERR_OK) {
        /* The core tops out at 1920x2032 and takes only the packed YUV420
         * layout above, so a rejection here is a configuration problem, not a
         * missing chip. */
        ESP_LOGE(TAG, "the H.264 encoder would not start for %" PRIu32 "x%" PRIu32, w, enc_h);
        printf("\n==== done - encoder failed ====\n");
        return;
    }
    if (esp_h264_enc_hw_get_param_hd(enc, &s_enc_param) != ESP_H264_ERR_OK) {
        s_enc_param = NULL;         /* the skip knob still works, rate control just will not follow */
        ESP_LOGW(TAG, "no encoder parameter handle - rate control cannot follow /skip");
    }
    apply_skip(VIDEO_FRAME_SKIP);
    ESP_LOGI(TAG, "H.264 %" PRIu32 "x%" PRIu32 " @ %d fps (1 in %d of %d), %d bit/s, "
             "GOP %d, QP %d-%d", w, enc_h, VIDEO_FPS / VIDEO_FRAME_SKIP,
             VIDEO_FRAME_SKIP, VIDEO_FPS, VIDEO_BITRATE, VIDEO_GOP,
             VIDEO_QP_MIN, VIDEO_QP_MAX);

    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "STREAMON failed");
        printf("\n==== done - camera would not stream ====\n");
        return;
    }

    /* static, not a stack local: app_main returns while the task is still
     * running, and the task dereferences this for the life of the program. */
    static struct encode_ctx ctx;
    ctx.fd = fd;
    ctx.buffer = buffer;
    ctx.frame_bytes = frame_bytes;
    ctx.enc = enc;
    ctx.enc_out = enc_out;
    ctx.enc_out_size = enc_out_size;

    encoder_warmup(&ctx, 30, "warm-up, radio off");
#if ENCODER_BITRATE_SWEEP
    encoder_freerun(&ctx, 20, false);
    encoder_freerun(&ctx, 20, true);
    encoder_bitrate_sweep(&ctx);
#endif

    /* WiFi after the camera is streaming, so the AE/AWB/AF convergence that
     * needs a few seconds of frames overlaps the association instead of
     * following it. */
    char ip[16] = {0};
    esp_err_t werr = imx_wifi_connect(ip, sizeof(ip));

    /*
     * Above the HTTP tasks, which run at 5. The encoder has a hard 36 ms
     * deadline per frame and the servers have none, so when the radio is busy
     * the camera should be the one that wins; it blocks on DQBUF between frames
     * anyway, so it cannot starve them.
     */
    if (xTaskCreatePinnedToCore(encode_task, "encode", 6144, &ctx,
                                6, NULL, tskNO_AFFINITY) != pdPASS) {
        ESP_LOGE(TAG, "could not start the encoder task");
        printf("\n==== done - encoder task failed ====\n");
        return;
    }

    if (werr != ESP_OK) {
        printf("\n==== done - camera up, WiFi failed ====\n");
        return;
    }
    if (!start_servers()) {
        printf("\n==== done - camera and WiFi up, server failed ====\n");
        return;
    }

    ESP_LOGI(TAG, "watch it at http://%s:%d/", ip, STREAM_PORT);
    ESP_LOGI(TAG, "  ffplay http://%s:%d/stream.h264", ip, STREAM_PORT);
    ESP_LOGI(TAG, "  ffplay http://%s:%d/stream.mp4", ip, STREAM_PORT);
    ESP_LOGI(TAG, "  instruments: http://%s/stats  /bench  /skip?n=", ip);

    /* The marker tools/capture.py waits for. Everything past this point happens
     * in the tasks, so a serial capture can stop here. */
    printf("\n==== done - streaming at http://%s:%d/ ====\n", ip, STREAM_PORT);
}
