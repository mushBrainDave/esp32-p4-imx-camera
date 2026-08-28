# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)

# Pull in the IMX219 add-on driver component from this repo's components/ dir.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(imx219_capture)
