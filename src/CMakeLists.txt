cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

PROJECT(kinectscan)

# CMake Modules

find_package(libfreenect REQUIRED)
find_package(OpenCV REQUIRED)
find_package(libusb-1.0 REQUIRED)

include_directories(${LIBFREENECT_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(camera)

add_definitions(-Wall)

add_library(util util/Mutex.cpp)
add_library(camera camera/KinectAdapter.cpp)
target_link_libraries(camera ${LIBFREENECT_LIBRARY})
target_link_libraries(camera ${OpenCV_LIBS})

add_executable(adapter_test test/KinectAdapterCmd.cpp)
target_link_libraries(adapter_test ${LIBFREENECT_LIBRARY})
target_link_libraries(adapter_test ${OpenCV_LIBS})
target_link_libraries(adapter_test ${LIBUSB_1_LIBRARIES})
target_link_libraries(adapter_test camera)
target_link_libraries(adapter_test util)
