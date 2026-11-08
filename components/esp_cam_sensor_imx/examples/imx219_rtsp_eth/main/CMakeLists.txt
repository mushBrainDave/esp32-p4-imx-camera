idf_component_register(
    SRCS "camera_main.c" "board_eth.c" "stream_bridge.c" "rtsp_socket.c" "rtsp_timer.c" "rtsp_wire.c" "rtsp_alloc.c" "h264_sdp.c" "profiles.c" "encoder_alloc.c" "fullview_scale.c" "fullview_worker.c" "image_request.c" "image_controls.c"
    INCLUDE_DIRS "."
    REQUIRES esp_mm esp_video esp_cam_sensor_imx esp_driver_uart esp_timer esp_eth esp_netif esp_http_server esp_media_protocols media_lib_sal nvs_flash
)

# Fail the build if the IMX219 ISP tuning config did not resolve. esp_ipa's own
# existence check is spelled message(FETAL_ERROR ...) - not a real CMake mode -
# so it prints and carries on, and a missing file shows up much later as a
# stream with no auto-exposure and no white balance rather than as an error.
# By the time this component is configured the path is final, so check it here.
idf_build_get_property(imx219_ipa_json IMX219_IPA_JSON_RESOLVED)
idf_build_get_property(imx219_project_dir PROJECT_DIR)
get_filename_component(imx219_ipa_abs "${imx219_ipa_json}"
    ABSOLUTE BASE_DIR "${imx219_project_dir}")
if(NOT EXISTS "${imx219_ipa_abs}")
    message(FATAL_ERROR
        "IMX219 ISP tuning config not found at:\n  ${imx219_ipa_abs}\n"
        "esp_ipa would silently skip it and the camera would run with no "
        "auto-exposure and no white balance. Check the ESP_IPA_JSON_CONFIG_FILE_PATH "
        "block in the project CMakeLists.txt.")
endif()

# Keep a pinned protocol library; isolate its Content-Length wire workaround.
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,-u,__wrap_esp_h264_aligned_malloc"
    "-Wl,--wrap=esp_h264_aligned_malloc"
    "-Wl,--wrap=heap_caps_free"
    "-Wl,-u,__wrap_media_lib_socket_send"
    "-Wl,--wrap=media_lib_socket_send"
    "-Wl,--wrap=media_lib_socket_write"
    "-Wl,--wrap=media_lib_socket_accept"
    "-Wl,--wrap=media_lib_socket_close"
    "-Wl,--wrap=media_lib_socket_listen"
    "-Wl,--wrap=media_lib_malloc"
    "-Wl,--wrap=media_lib_calloc"
    "-Wl,--wrap=media_lib_free"
    "-Wl,--wrap=esp_timer_create"
    "-Wl,--wrap=esp_timer_delete")

# Keep the pixel loop optimized even with the development SDK debug baseline.
set_source_files_properties(fullview_scale.c PROPERTIES COMPILE_OPTIONS "-O3")
