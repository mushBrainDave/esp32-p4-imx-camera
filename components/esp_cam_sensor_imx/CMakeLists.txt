# esp_cam_sensor_imx — Sony IMX-series sensor drivers for the ESP32-P4
# esp_cam_sensor framework. Standalone add-on: plugs into esp_cam_sensor's
# auto-detect array via the ".esp_cam_sensor_detect_fn" linker section
# (esp_cam_sensor's linker.lf gathers this section from *all* archives).

set(srcs)
set(include_dirs)
set(priv_include_dirs)

if(CONFIG_CAMERA_IMX219)
    list(APPEND srcs "sensors/imx219/imx219.c")
    list(APPEND include_dirs "sensors/imx219/include")
    list(APPEND priv_include_dirs "sensors/imx219/private_include")
endif()

if(CONFIG_CAMERA_IMX708)
    list(APPEND srcs "sensors/imx708/imx708.c")
    list(APPEND include_dirs "sensors/imx708/include")
    list(APPEND priv_include_dirs "sensors/imx708/private_include")
endif()

# Autofocus actuators. These are separate chips from the sensor, on their own
# I2C addresses, with their own detect array (".esp_cam_motor_detect_fn") - the
# DW9807 is the VCM inside the Raspberry Pi Camera Module 3 alongside the IMX708.
if(CONFIG_CAM_MOTOR_DW9807)
    list(APPEND srcs "motors/dw9807/dw9807.c")
    list(APPEND include_dirs "motors/dw9807/include")
    list(APPEND priv_include_dirs "motors/dw9807/private_include")
endif()

# ISP tuning (IPA) configs live in sensors/<name>/cfg/<name>_default.json, but
# they CANNOT be registered from here: esp_ipa reads ESP_IPA_JSON_CONFIG_FILE_PATH
# while its own CMakeLists is processed, which has already happened by the time a
# component's CMakeLists runs. The app must register them in its project
# CMakeLists, after include(project.cmake) and before project() - see
# examples/imx708_snapshot/CMakeLists.txt. Without that registration
# esp_ipa_pipeline_get_config("IMX708") returns NULL, esp_video logs
# "failed to get configuration to initialize ISP controller", and you get no
# auto-exposure and no white balance.

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${include_dirs}
    PRIV_INCLUDE_DIRS ${priv_include_dirs}
    REQUIRES esp_cam_sensor
    PRIV_REQUIRES esp_driver_gpio esp_sccb_intf esp_timer
)

# Force the linker to keep each sensor's detect object so its entry lands in the
# esp_cam_sensor auto-detect array. Without this the linker drops the object,
# since nothing in the app references it directly.
if(CONFIG_CAMERA_IMX219)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u imx219_detect")
endif()

if(CONFIG_CAMERA_IMX708)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u imx708_detect")
endif()

if(CONFIG_CAM_MOTOR_DW9807)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u dw9807_detect")
endif()
