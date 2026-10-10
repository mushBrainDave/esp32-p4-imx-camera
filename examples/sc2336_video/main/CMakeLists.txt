idf_component_register(
    SRCS "sc2336_video_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_video imx_serial_img esp_driver_gpio esp_driver_i2c esp_timer
)
