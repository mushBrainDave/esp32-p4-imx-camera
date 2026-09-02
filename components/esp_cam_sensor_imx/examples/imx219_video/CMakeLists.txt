# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# This example ships inside the esp_cam_sensor_imx component, so it builds two
# ways, and the driver arrives from a different place in each:
#
#   in-repo    main/idf_component.yml carries an override_path pointing at the
#              component root two directories up, so the solver uses this
#              working tree instead of downloading a published version.
#
#   from the   `idf.py create-project-from-example` copies this directory out
#   registry   on its own and the driver is resolved normally, landing in
#              managed_components/mushbraindave__esp_cam_sensor_imx.
#
# Only the in-repo layout can be detected here: managed_components/ does not
# exist yet at this point on a clean build - the dependency solver runs inside
# project() - whereas the component root either is in this checkout or is not.
set(imx_component_root "${CMAKE_CURRENT_LIST_DIR}/../..")
if(EXISTS "${imx_component_root}/idf_component.yml")
    set(imx219_ipa_json "${imx_component_root}/sensors/imx219/cfg/imx219_default.json")
else()
    set(imx219_ipa_json
        "managed_components/mushbraindave__esp_cam_sensor_imx/sensors/imx219/cfg/imx219_default.json")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Hand the IMX219's ISP tuning config to esp_ipa. This has to sit between
# project.cmake and project(): esp_ipa reads the property while its own
# CMakeLists is processed, which is too late for a component to set it.
#
# Only the string is needed now - esp_ipa resolves it against the project
# directory and checks it exists when *it* is configured, by which time the
# component manager has already populated managed_components/. So the registry
# path above is valid even on a clean first build.
#
# Get it wrong and nothing complains: esp_ipa's existence check calls
# message(FETAL_ERROR ...), which is not a real CMake mode, so a missing file
# prints a line and the build carries on to produce a stream with no
# auto-exposure and no white balance. main/CMakeLists.txt re-checks the
# resolved path and fails the build instead.
idf_build_set_property(ESP_IPA_JSON_CONFIG_FILE_PATH "${imx219_ipa_json}" APPEND)
idf_build_set_property(IMX219_IPA_JSON_RESOLVED "${imx219_ipa_json}")

# ---------------------------------------------------------------------------
# ABI fix: make esp_ipa_stats_t match the prebuilt esp_ipa binary.
#
# esp_ipa ships as a closed .a. The one selected for us (lib/esp32p4/v5.4-) was
# compiled with the AWB sub-window grid present, i.e. with
# ISP_AWB_WINDOW_X_NUM / _Y_NUM defined as 5. Those macros only exist in
# ESP-IDF >= v5.4.4, so on v5.4.0 esp_ipa_types.h collapses
# esp_ipa_stats_t::awb_subwin to a [0][0] array - 400 bytes shorter than the
# library expects - and every member after it sits at the wrong offset.
#
# This sensor has no autofocus, so the member that suffered worst on the IMX708
# (af_stats, which the library read from past the end of the struct) is not in
# play here. The block is kept anyway: the mismatch is a property of the
# library and the IDF version, not of the sensor, and leaving the layouts
# disagreeing would be relying on the surviving members happening to sit before
# the hole. The tell either way is the "excess elements in array initializer"
# warnings from the generated esp_video_ipa_config.c - they mean the generator
# emitted a 5x5 subwin_weight into a float[0][0], and their absence is the
# confirmation that the layouts agree.
#
# This has to be global - esp_video writes the struct, the generated IPA config
# declares it, and the library reads it, so all three must agree.
#
# Only needed below v5.4.4; from there ESP-IDF defines the macros itself and
# the guard steps aside rather than defining them twice.
if(IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR EQUAL 4 AND IDF_VERSION_PATCH LESS 4)
    idf_build_set_property(COMPILE_DEFINITIONS "ISP_AWB_WINDOW_X_NUM=5" APPEND)
    idf_build_set_property(COMPILE_DEFINITIONS "ISP_AWB_WINDOW_Y_NUM=5" APPEND)
endif()

project(imx219_video)
