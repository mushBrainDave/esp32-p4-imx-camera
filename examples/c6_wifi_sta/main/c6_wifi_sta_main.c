/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Associate with an access point through the ESP32-C6 and prove the link
 * carries real traffic.
 *
 * `c6_link_check` established that the P4 <-> C6 SDIO path is alive: the radio
 * enumerates and a scan returns real APs. That stops one step short of a
 * usable network, because scanning is receive-only. This example takes the
 * next step - associate, run DHCP, then open a TCP connection to a host on the
 * internet - which exercises transmit, the 4-way handshake, the routed path
 * and DNS. Only after that is it fair to say the board is on the network.
 *
 * Everything above the transport is stock ESP-IDF: esp_wifi_remote forwards the
 * esp_wifi calls to the C6, but esp_netif, LwIP and DHCP all run on the P4, so
 * the sockets below are ordinary sockets.
 *
 * Failures are reported, not asserted away. The disconnect reason code is the
 * whole diagnostic here - it is what separates "wrong password" from "that
 * SSID is 5 GHz only", and an abort would throw it away.
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/*
 * Written by hand from wifi_credentials.h.example and kept out of git. Absent,
 * the build still succeeds and app_main says what to do - a missing secret
 * should not look like a broken example.
 */
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#endif

/*
 * Empty defaults rather than an #error, and checked at run time rather than
 * with #if: this way the connect path is compiled whether or not the header
 * exists, so a build without credentials still type-checks all of it.
 */
#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID ""
#endif
#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD ""
#endif

static const char *TAG = "c6_wifi_sta";

/* Enough to distinguish a slow DHCP from an AP that is not answering, without
 * making a wrong password take a minute to report. */
#define MAX_RETRIES        5
#define CONNECT_TIMEOUT_MS 30000

/* The reachability probe. Port 80 rather than 443 deliberately: a TLS
 * handshake would test mbedtls, and what is under test here is the radio. */
#define PROBE_HOST "example.com"
#define PROBE_PORT "80"

#define CONNECTED_BIT BIT0
#define FAILED_BIT    BIT1

static EventGroupHandle_t s_events;
static int s_retries;
static uint8_t s_last_reason;

/*
 * The reason codes that actually turn up on a first bring-up. Everything else
 * prints as a number - the point is to name the handful that have a specific
 * fix, not to mirror the whole enum.
 */
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
        /* Connect from the event, not from app_main: esp_wifi_connect before
         * the driver has finished starting returns ESP_ERR_WIFI_NOT_STARTED. */
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
        ESP_LOGI(TAG, "got IP " IPSTR "  netmask " IPSTR "  gateway " IPSTR,
                 IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.netmask),
                 IP2STR(&e->ip_info.gw));
        xEventGroupSetBits(s_events, CONNECTED_BIT);
    }
}

/*
 * DNS, then a TCP connect, then one HTTP exchange. Any of the three failing on
 * its own is informative: DNS failing after a good DHCP lease points at the
 * DNS server the AP handed out, a connect failing after DNS points at the
 * route, and a request that connects but returns nothing points at the link
 * dropping mid-flight rather than at association.
 */
static bool probe_internet(void)
{
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;

    ESP_LOGI(TAG, "resolving %s ...", PROBE_HOST);
    int err = getaddrinfo(PROBE_HOST, PROBE_PORT, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGE(TAG, "DNS lookup failed (err %d) - associated, but name "
                      "resolution is not working", err);
        return false;
    }

    struct in_addr *addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
    ESP_LOGI(TAG, "%s resolved to %s", PROBE_HOST, inet_ntoa(*addr));

    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        freeaddrinfo(res);
        return false;
    }

    /* Without a timeout a black-holed route hangs this task indefinitely, and
     * the run would look like a crash rather than a failed probe. */
    struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    bool ok = false;
    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "connect() failed: errno %d - DNS works but the route "
                      "out does not", errno);
    } else {
        ESP_LOGI(TAG, "TCP connected to %s:%s", PROBE_HOST, PROBE_PORT);
        const char *req = "GET / HTTP/1.0\r\nHost: " PROBE_HOST "\r\n"
                          "Connection: close\r\n\r\n";
        if (send(sock, req, strlen(req), 0) < 0) {
            ESP_LOGE(TAG, "send() failed: errno %d", errno);
        } else {
            char buf[128] = { 0 };
            int n = recv(sock, buf, sizeof(buf) - 1, 0);
            if (n <= 0) {
                ESP_LOGE(TAG, "recv() returned %d, errno %d", n, errno);
            } else {
                /* Just the status line - this is a reachability check, not an
                 * HTTP client. */
                char *eol = strpbrk(buf, "\r\n");
                if (eol) {
                    *eol = '\0';
                }
                ESP_LOGI(TAG, "HTTP response: %s", buf);
                ESP_LOGI(TAG, "received %d bytes - the path out is real", n);
                ok = true;
            }
        }
    }

    close(sock);
    freeaddrinfo(res);
    return ok;
}

static void log_ap_details(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        return;
    }
    ESP_LOGI(TAG, "associated with \"%s\"  bssid %02x:%02x:%02x:%02x:%02x:%02x",
             (const char *)ap.ssid, ap.bssid[0], ap.bssid[1], ap.bssid[2],
             ap.bssid[3], ap.bssid[4], ap.bssid[5]);
    ESP_LOGI(TAG, "channel %u  rssi %d dBm  authmode %d  phy %s%s%s%s",
             ap.primary, ap.rssi, ap.authmode,
             ap.phy_11b ? "b" : "", ap.phy_11g ? "g" : "",
             ap.phy_11n ? "n" : "", ap.phy_11ax ? "/ax" : "");
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C6 station connect over SDIO");

    if (WIFI_STA_SSID[0] == '\0') {
        ESP_LOGE(TAG, "no credentials compiled in.");
        ESP_LOGE(TAG, "copy main/wifi_credentials.h.example to "
                      "main/wifi_credentials.h, fill in SSID and password, "
                      "and reflash. That file is gitignored.");
        ESP_LOGE(TAG, "if you just created it and are seeing this anyway, the "
                      "build reused a stale object - the header is included "
                      "behind __has_include, so it was not in the depfile. "
                      "Run 'idf.py fullclean' or touch the source.");
        printf("\n==== done - no credentials ====\n");
        return;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_ip_event, NULL, NULL));

    /* As in c6_link_check: a failure here is about the C6, not about us, so
     * report it instead of aborting on it. */
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s - run c6_link_check first",
                 esp_err_to_name(err));
        printf("\n==== done - link DOWN ====\n");
        return;
    }

    wifi_config_t sta_cfg = { 0 };
    /* strncpy into fixed-size arrays, and the struct was zeroed above, so the
     * unused tail the driver reads stays zeroed. */
    strncpy((char *)sta_cfg.sta.ssid, WIFI_STA_SSID, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, WIFI_STA_PASSWORD,
            sizeof(sta_cfg.sta.password) - 1);
    /* Leave threshold.authmode at its default (open) so an open or WPA-only
     * network still associates; raising it silently filters APs out. H2E both
     * covers WPA3 APs that require hash-to-element. */
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
            ESP_LOGE(TAG, "timed out after %d ms with no result at all - "
                          "neither an IP nor a disconnect event",
                     CONNECT_TIMEOUT_MS);
        }
        printf("\n==== done - NOT connected ====\n");
        return;
    }

    log_ap_details();

    esp_netif_dns_info_t dns;
    if (esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK) {
        ESP_LOGI(TAG, "dns " IPSTR, IP2STR(&dns.ip.u_addr.ip4));
    }

    /* Latency over throughput, with an eye on what this link is for next -
     * streaming frames off the board. Power save has the AP buffer traffic
     * between beacons, which shows up as bursty round-trips. */
    esp_wifi_set_ps(WIFI_PS_NONE);

    bool reachable = probe_internet();

    printf("\n==== done - %s ====\n",
           reachable ? "CONNECTED, internet reachable"
                     : "CONNECTED, but no route out");
}
