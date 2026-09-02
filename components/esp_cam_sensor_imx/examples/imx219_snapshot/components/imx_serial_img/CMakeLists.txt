# SPDX-License-Identifier: Apache-2.0
idf_component_register(
    SRCS "imx_serial_img.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart
)
