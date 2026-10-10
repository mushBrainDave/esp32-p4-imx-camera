# SPDX-License-Identifier: Apache-2.0
#
# Bench tool, not part of the component release: records H.264 from the
# Wireless-Tag WTDK-CAM-2336 (SmartSens SC2336) on a WT9932P4-TINY, to prove
# the capture -> ISP -> H.264 path on ESP32-P4 v3.x silicon with a camera the
# board is known to be wired for. The SC2336 driver and its ISP tuning file
# both come from espressif/esp_cam_sensor (its project_include.cmake picks the
# tuning file for the chip revision), so nothing here sets them.
#
# Needs ESP-IDF v5.5 or later: v5.4 cannot target v3.x silicon.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sc2336_video)
