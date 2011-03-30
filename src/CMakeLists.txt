cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

PROJECT(kinectscan)

# CMake Modules

find_package(libfreenect REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${LIBFREENECT_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIR})

include_directories(${CMAKE_SOURCE_DIR})

add_definitions(-Wall)

add_library(util util/Mutex.cpp)
add_library(camera camera/KinectAdapter.cpp)
target_link_libraries(camera ${LIBFREENECT_LIBRARY})
target_link_libraries(camera ${OpenCV_LIBS})
