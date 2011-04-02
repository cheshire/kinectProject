# Third-party dependencies.

find_package(libfreenect REQUIRED)
find_package(OpenCV REQUIRED)
find_package(libusb-1.0 REQUIRED)

include_directories(${LIBFREENECT_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIR})

set(third_party_include ${LIBFREENECT_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})
set(third_party_lib ${LIBFREENECT_LIBRARY} ${OpenCV_LIBS})

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
target_link_libraries(camera ${THIRD_PARTY_LIB})

add_executable(kinect_view examples/kinect_view.cc)
target_link_libraries(kinect_view ${LIBRARIES} ${THIRD_PARTY_LIB})