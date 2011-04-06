# Third-party dependencies.

find_package(libfreenect REQUIRED)
find_package(OpenCV REQUIRED)
find_package(libusb-1.0 REQUIRED)
find_package(GFlags REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${LIBFREENECT_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

set(third_party_include ${LIBFREENECT_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})
set(third_party_lib ${LIBFREENECT_LIBRARY} ${OpenCV_LIBS} ${GFlags_LIBS})

# General compilation flags.

add_definitions(-Wall -I${CMAKE_SOURCE_DIR}/src)

# Source directories.

file(GLOB CAMERA_LIB "camera/*.cc")
file(GLOB UTIL_LIB "util/*.cc")
include_directories(camera)
include_directories(util)

# Libraries

add_library(util ${UTIL_LIB})

add_library(camera ${CAMERA_LIB})
target_link_libraries(camera util ${THIRD_PARTY_LIB})

add_executable(kinect_view examples/kinect_view.cc)
target_link_libraries(kinect_view camera util)
target_link_libraries(kinect_view ${LIBFREENECT_LIBRARY} ${LIBUSB_1_LIBRARIES} ${OpenCV_LIBS} ${GFlags_LIBS})
