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

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${include_dirs}
    PRIV_INCLUDE_DIRS ${priv_include_dirs}
    REQUIRES esp_cam_sensor
    PRIV_REQUIRES esp_driver_gpio esp_sccb_intf
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
