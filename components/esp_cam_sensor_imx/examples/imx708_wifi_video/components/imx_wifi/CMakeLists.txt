# SPDX-License-Identifier: Apache-2.0
idf_component_register(
    SRCS "imx_wifi.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif esp_event nvs_flash
)

# wifi_credentials.h is optional and pulled in behind __has_include, so it
# cannot appear in the depfile when it does not exist - creating it for the
# first time would otherwise leave a stale object in place and the firmware
# would still report "no credentials". This covers edits to an existing header;
# the first appearance needs `idf.py fullclean`, which is what the runtime
# message says.
set(creds "${CMAKE_CURRENT_LIST_DIR}/include/wifi_credentials.h")
if(EXISTS "${creds}")
    set_source_files_properties(imx_wifi.c PROPERTIES OBJECT_DEPENDS "${creds}")
else()
    message(STATUS "imx_wifi: no wifi_credentials.h - building without credentials")
endif()
