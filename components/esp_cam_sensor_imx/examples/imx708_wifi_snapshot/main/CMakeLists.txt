idf_component_register(
    SRCS "imx708_wifi_snapshot_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_video esp_cam_sensor_imx imx_wifi esp_driver_jpeg esp_http_server esp_timer
)

# Fail the build if the IMX708 ISP tuning config did not resolve. esp_ipa's own
# existence check is spelled message(FETAL_ERROR ...) - not a real CMake mode -
# so it prints and carries on, and a missing file shows up much later as a
# stream with no auto-exposure and no white balance rather than as an error.
# By the time this component is configured the path is final, so check it here.
idf_build_get_property(imx708_ipa_json IMX708_IPA_JSON_RESOLVED)
idf_build_get_property(imx708_project_dir PROJECT_DIR)
get_filename_component(imx708_ipa_abs "${imx708_ipa_json}"
    ABSOLUTE BASE_DIR "${imx708_project_dir}")
if(NOT EXISTS "${imx708_ipa_abs}")
    message(FATAL_ERROR
        "IMX708 ISP tuning config not found at:\n  ${imx708_ipa_abs}\n"
        "esp_ipa would silently skip it and the camera would run with no "
        "auto-exposure and no white balance. Check the ESP_IPA_JSON_CONFIG_FILE_PATH "
        "block in the project CMakeLists.txt.")
endif()
