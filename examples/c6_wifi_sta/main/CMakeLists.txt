idf_component_register(
    SRCS "c6_wifi_sta_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_netif esp_event nvs_flash lwip
)

# The credentials header is pulled in behind __has_include, so it is invisible
# to the depfile when it does not exist: creating it for the first time does
# NOT invalidate an object built without it, and the next build silently keeps
# the "no credentials" path. Declaring the dependency here fixes edits to an
# existing header. The first appearance still needs a reconfigure, which is why
# the app also says so at run time.
set(creds "${CMAKE_CURRENT_LIST_DIR}/wifi_credentials.h")
if(EXISTS "${creds}")
    set_source_files_properties(c6_wifi_sta_main.c
        PROPERTIES OBJECT_DEPENDS "${creds}")
else()
    message(STATUS "c6_wifi_sta: no wifi_credentials.h - building without credentials")
endif()
