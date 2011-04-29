find_path(LIBOPENNI_INCLUDE_DIRS
  NAMES
    XnOpenNI.h
  PATHS
    /usr/include
    /usr/include/ni
    /usr/local/include
    /usr/local/include/ni
)

find_library(LIBOPENNI_LIBRARIES
  NAMES
    libOpenNI.so
  PATHS
    /usr/lib
    /usr/lib32
    /usr/lib64
    /usr/local/lib
    /usr/local/lib32
    /usr/local/lib64
)


message(STATUS "Found libOpenNI:")
message(STATUS " - Includes: ${LIBOPENNI_INCLUDE_DIRS}")
message(STATUS " - Libraries: ${LIBOPENNI_LIBRARIES}")