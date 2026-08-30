/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Does the ESP32-P4 <-> ESP32-C6 link come up at all?
 *
 * The P4 has no radio. WiFi here means esp_wifi_remote forwarding every
 * esp_wifi call over SDIO to a C6 running esp_hosted "slave" firmware. That
 * makes the first question a binary one - is there a slave on the other end of
 * the bus - and it is worth answering before any application code exists,
 * because a "no" has three quite different causes: wrong pins, no slave
 * firmware flashed, or a host/slave version mismatch.
 *
 * So this does the smallest thing that exercises the whole path: bring up
 * WiFi and scan. A scan needs the radio to actually work, not merely to
 * enumerate, so a returned AP list is real evidence rather than a driver
 * politely accepting a config.
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "c6_link_check";

#define MAX_APS 20

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C6 link check over SDIO");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /*
     * esp_wifi_init is where the transport is brought up, so it is the first
     * call that can fail because of the C6 rather than because of us. Do not
     * ESP_ERROR_CHECK it - a failure here is the interesting result, and
     * aborting would throw away the log line that says why.
     */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "no usable slave on the SDIO bus - check, in order: "
                      "slave firmware present on the C6, esp_hosted host/slave "
                      "versions matching, then the pin map");
        printf("\n==== done - link DOWN ====\n");
        return;
    }
    ESP_LOGI(TAG, "esp_wifi_init OK - the C6 answered");

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "esp_wifi_start OK");

    uint8_t mac[6] = {0};
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
        /* A real MAC read back over the link is a second, independent sign
         * that the far side is a genuine radio and not a stub. */
        ESP_LOGI(TAG, "C6 station MAC %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    ESP_LOGI(TAG, "scanning...");
    err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "scan failed: %s", esp_err_to_name(err));
        printf("\n==== done - link UP but scan failed ====\n");
        return;
    }

    /* Ask for the total before fetching records: get_ap_records frees the
     * scan list as it copies it out, after which get_ap_num reports 0. */
    uint16_t total = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&total));

    uint16_t n = MAX_APS;
    static wifi_ap_record_t aps[MAX_APS];
    memset(aps, 0, sizeof(aps));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&n, aps));
    ESP_LOGI(TAG, "scan found %u AP(s), showing %u:", total, n);
    for (uint16_t i = 0; i < n; i++) {
        ESP_LOGI(TAG, "  %-32s ch %2u  rssi %4d",
                 (const char *)aps[i].ssid, aps[i].primary, aps[i].rssi);
    }

    printf("\n==== done - link UP ====\n");
}
