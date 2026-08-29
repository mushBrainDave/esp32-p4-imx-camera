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
project(imx708_snapshot)
