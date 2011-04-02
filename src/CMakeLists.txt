cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

PROJECT(kinectscan)

# Third-party dependencies.

find_package(libfreenect REQUIRED)
find_package(OpenCV REQUIRED)
find_package(libusb-1.0 REQUIRED)

include_directories(${LIBFREENECT_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR})

set(third_party ${LIBFREENECT_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR} ${CMAKE_SOURCE_DIR})

# General compilation flags.

add_definitions(-Wall)

# Source directories.

aux_source_directory(camera ${CAMERA_LIB_DIR})
include_directories(camera)
aux_source_directory(util ${UTIL_LIB_DIR})
include_directories(util)

set(libraries ${CAMERA_LIB_DIR} ${UTIL_LIB_DIR})

# Libraries

add_library(util ${UTIL_LIB_DIR})

add_library(camera ${CAMERA_LIB_DIR})
target_link_libraries(camera ${THIRD_PARTY})