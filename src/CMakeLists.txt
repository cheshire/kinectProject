# Third-party dependencies.

find_package(libfreenect REQUIRED)
find_package(OpenCV REQUIRED)
find_package(libusb-1.0 REQUIRED)
find_package(GFlags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenGL)
find_package(OpenNI REQUIRED)

include_directories(${LIBFREENECT_INCLUDE_DIR})
include_directories(${GLOG_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${LIBOPENNI_INCLUDE_DIRS})

set(third_party_include ${LIBFREENECT_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR} ${GLOG_INCLUDE_DIRS} ${LIBOPENNI_INCLUDE_DIRS})
set(third_party_lib ${LIBFREENECT_LIBRARY} ${OpenCV_LIBS} ${GFlags_LIBS} ${GLOG_LIBRARIES} ${LIBOPENNI_LIBRARIES})

# General compilation flags.
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O2 -g")
add_definitions(-Wall -I${CMAKE_SOURCE_DIR}/src)

# Source directories.

file(GLOB CAMERA_LIB "camera/*.cc")
file(GLOB UTIL_LIB "util/*.cc")
file(GLOB MESH_LIB "mesh/*.cc")
file(GLOB ORIENTATION_DETECTOR_LIB "orientation_detector/*.cc")
include_directories(camera)
include_directories(util)
include_directories(mesh)
include_directories(orientation_detector)

# Libraries
add_library(orientation_detector ${ORIENTATION_DETECTOR_LIB})

add_library(util ${UTIL_LIB})

add_library(camera ${CAMERA_LIB})
target_link_libraries(camera util ${THIRD_PARTY_LIB})

add_library(mesh ${MESH_LIB})
target_link_libraries(mesh camera util ${THIRD_PARTY_LIB} ${OPENGL_LIBRARY})

add_executable(kinect_view examples/kinect_view.cc)
target_link_libraries(kinect_view camera util mesh orientation_detector)
target_link_libraries(kinect_view ${LIBFREENECT_LIBRARY} ${LIBUSB_1_LIBRARIES} ${OpenCV_LIBS} ${GFlags_LIBS} ${GLOG_LIBRARIES} ${OPENGL_LIBRARY} ${LIBOPENNI_LIBRARIES})

