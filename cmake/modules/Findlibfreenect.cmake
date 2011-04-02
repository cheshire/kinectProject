find_path(LIBFREENECT_INCLUDE_DIR
  NAMES
    libfreenect.h
  PATHS
    /usr/include
    /usr/local/include
)

find_library(LIBFREENECT_LIBRARY
  NAMES
    libfreenect.a
  PATHS
    /usr/lib
    /usr/lib32
    /usr/lib64
    /usr/local/lib
    /usr/local/lib32
    /usr/local/lib64
)

set(LIBFREENECT_INCLUDE_DIRS
  ${LIBFREENECT_INCLUDE_DIRS}
)

SET(LIBFREENECT_LIBRARIES
  ${LIBFREENECT_LIBRARY}
)

if (LIBFREENECT_INCLUDE_DIRS AND LIBFREENECT_LIBRARIES)
  set (LIBFREENECT_FOUND TRUE)
endif (LIBFREENECT_INCLUDE_DIRS AND LIBFREENECT_LIBRARIES)

