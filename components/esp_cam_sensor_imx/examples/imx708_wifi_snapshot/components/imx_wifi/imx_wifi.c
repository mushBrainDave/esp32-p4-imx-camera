/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See imx_wifi.h. The reasoning behind the non-obvious choices here was worked
 * out in examples/c6_wifi_sta and is written up in docs/esp32c6-bringup.md.
 */
#include <string.h>
#include "imx_wifi.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#endif

/*
 * Empty defaults, checked at run time rather than with #if, so the connect path
 * is compiled whether or not the header exists.
 *
 * Note the header is pulled in behind __has_include, which means it cannot be
 * in the depfile when it does not exist: creating it for the first time does
 * NOT invalidate an object built without it. CMakeLists declares it as an
 * OBJECT_DEPENDS when present; the first appearance still needs a fullclean.
 */
#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID ""
#endif
#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD ""
#endif

static const char *TAG = "imx_wifi";

#define MAX_RETRIES        5
#define CONNECT_TIMEOUT_MS 30000

#define CONNECTED_BIT BIT0
#define FAILED_BIT    BIT1

static EventGroupHandle_t s_events;
static int s_retries;
static uint8_t s_last_reason;
static esp_netif_t *s_netif;
static char s_ip[16];

static const char *reason_hint(uint8_t reason)
{
    switch (reason) {
    case WIFI_REASON_NO_AP_FOUND:
        return "SSID not seen. Check spelling, and remember the C6 is "
               "2.4 GHz only - a 5 GHz-only SSID is invisible to it";
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        return "authentication failed - almost always a wrong password";
    case WIFI_REASON_ASSOC_FAIL:
        return "the AP refused the association (MAC filtering? band steering?)";
    case WIFI_REASON_AUTH_EXPIRE:
    case WIFI_REASON_ASSOC_EXPIRE:
        return "the AP dropped us after associating - weak signal, or the AP "
               "aged the session out";
    default:
        return "see WIFI_REASON_* in esp_wifi_types.h";
    }
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_STA_START) {
        /* From the event, not from the caller: connecting before the driver has
         * finished starting returns ESP_ERR_WIFI_NOT_STARTED. */
        esp_wifi_connect();
        return;
    }

    if (id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        s_last_reason = e->reason;
        if (s_retries < MAX_RETRIES) {
            s_retries++;
            ESP_LOGW(TAG, "disconnected, reason %u - retry %d/%d",
                     e->reason, s_retries, MAX_RETRIES);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "giving up after %d attempts, last reason %u: %s",
                     MAX_RETRIES, e->reason, reason_hint(e->reason));
            xEventGroupSetBits(s_events, FAILED_BIT);
        }
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "got IP %s  gateway " IPSTR, s_ip, IP2STR(&e->ip_info.gw));
        xEventGroupSetBits(s_events, CONNECTED_BIT);
    }
}

bool imx_wifi_have_credentials(void)
{
    return WIFI_STA_SSID[0] != '\0';
}

static void log_ap_details(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        return;
    }
    ESP_LOGI(TAG, "associated with \"%s\"  channel %u  rssi %d dBm  phy %s%s%s%s",
             (const char *)ap.ssid, ap.primary, ap.rssi,
             ap.phy_11b ? "b" : "", ap.phy_11g ? "g" : "",
             ap.phy_11n ? "n" : "", ap.phy_11ax ? "/ax" : "");
}

esp_err_t imx_wifi_connect(char *ip_out, size_t ip_len)
{
    if (!imx_wifi_have_credentials()) {
        ESP_LOGE(TAG, "no credentials compiled in. Copy "
                      "components/imx_wifi/include/wifi_credentials.h.example "
                      "to wifi_credentials.h beside it and fill it in.");
        ESP_LOGE(TAG, "if you just created it and are seeing this anyway, run "
                      "'idf.py fullclean' - the header is behind __has_include, "
                      "so it was not in the depfile and a stale object was reused.");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_events = xEventGroupCreate();
    if (!s_events) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif = esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_ip_event, NULL, NULL));

    /* A failure here is the C6 not answering, not a network problem. Report it
     * as such instead of aborting - the distinction is the whole point. */
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s - the C6 did not answer. "
                      "Run examples/c6_link_check.", esp_err_to_name(err));
        return ESP_ERR_WIFI_NOT_INIT;
    }

    wifi_config_t sta_cfg = { 0 };
    strncpy((char *)sta_cfg.sta.ssid, WIFI_STA_SSID, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, WIFI_STA_PASSWORD,
            sizeof(sta_cfg.sta.password) - 1);
    /* threshold.authmode stays at its default (open): raising it to WPA2_PSK,
     * as Espressif's examples do, silently filters weaker APs out and reports
     * it as NO_AP_FOUND, which reads as a missing AP rather than a policy. */
    sta_cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "connecting to \"%s\" ...", WIFI_STA_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_events, CONNECTED_BIT | FAILED_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));
    if (!(bits & CONNECTED_BIT)) {
        if (bits & FAILED_BIT) {
            ESP_LOGE(TAG, "could not associate. last reason %u: %s",
                     s_last_reason, reason_hint(s_last_reason));
        } else {
            ESP_LOGE(TAG, "timed out after %d ms with no result at all",
                     CONNECT_TIMEOUT_MS);
        }
        return ESP_ERR_TIMEOUT;
    }

    log_ap_details();

    /*
     * Latency and steady throughput over idle power. Power save has the AP
     * buffer traffic between beacons, which turns a stream of frames into
     * bursts - the opposite of what this component exists to carry.
     */
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (ip_out && ip_len) {
        strncpy(ip_out, s_ip, ip_len - 1);
        ip_out[ip_len - 1] = '\0';
    }
    return ESP_OK;
}
