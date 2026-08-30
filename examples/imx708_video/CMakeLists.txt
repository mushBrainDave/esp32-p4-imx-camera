# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Hand the IMX708's ISP tuning config to esp_ipa. This has to sit between
# project.cmake and project(): esp_ipa reads the property as its own CMakeLists
# is processed, so setting it from a component is too late to be seen.
idf_build_set_property(ESP_IPA_JSON_CONFIG_FILE_PATH
    "${CMAKE_CURRENT_LIST_DIR}/../../components/esp_cam_sensor_imx/sensors/imx708/cfg/imx708_default.json"
    APPEND)
# ---------------------------------------------------------------------------
# ABI fix: make esp_ipa_stats_t match the prebuilt esp_ipa binary.
#
# esp_ipa ships as a closed .a. The one selected for us (lib/esp32p4/v5.4-)
# was compiled with the AWB sub-window grid present, i.e. with
# ISP_AWB_WINDOW_X_NUM / _Y_NUM defined as 5. Those macros only exist in
# ESP-IDF >= v5.4.4 (see esp_video_caps.h, which gates AWB subwindow support on
# exactly that version). On v5.4.0 they are undefined, so esp_ipa_types.h
# collapses esp_ipa_stats_t::awb_subwin to a [0][0] array - 400 bytes shorter
# than the library expects, which puts af_stats at offset 196 in a 224-byte
# struct while the library reads it from 596. Autofocus then scores unrelated
# heap memory and always settles on its first scan point.
#
# This has to be global - esp_video writes the struct, the generated IPA config
# declares it, and the library reads it, so all three must agree. The real fix
# is ESP-IDF >= v5.4.4, which defines these itself; remove this block on
# upgrading, or the macros will be defined twice.
#
# See examples/imx708_snapshot/CMakeLists.txt for the full write-up.
idf_build_set_property(COMPILE_DEFINITIONS "ISP_AWB_WINDOW_X_NUM=5" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "ISP_AWB_WINDOW_Y_NUM=5" APPEND)

project(imx708_video)
