# Copied from http://code.google.com/p/opencv-feature-tracker/source/browse/cmake_modules/FindGFlags.cmake?r=f804b03e704147e65183c19a50f57abedb22f45c
#
# - Try to find GFlags
#
# The following are set after configuration is done: 
#  GFlags_FOUND
#  GFlags_INCLUDE_DIRS
#  GFlags_LIBS
#  GFlags_MODEL_DIRS
#  GFlags_LIBRARY_DIRS
cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0011 OLD)

FIND_PATH(GFlags_LIBRARY_DIRS
        libgflags.a
        HINTS
        /usr/local/lib
)

IF(GFlags_LIBRARY_DIRS)
  FIND_PATH(GFlags_INCLUDE_DIRS
          gflags/gflags.h
          HINTS
          /usr/local/include
          ${GFlags_ROOT_DIR}/src
  )

    # set up include and link directory
    include_directories(
        ${GFlags_INCLUDE_DIRS}
    )
    link_directories(${GFlags_LIBRARY_DIRS})

    SET(GFlags_LIBS
        ${GFlags_lib}
    )

    SET(GFlags_FOUND true)

    MARK_AS_ADVANCED(
        GFlags_INCLUDE_DIRS
        )
ELSE(GFlags_LIBRARY_DIRS)
    FIND_PATH(GFlags_ROOT_DIR
        src
    )
    MARK_AS_ADVANCED(GFlags_ROOT_DIR)
    MESSAGE(SEND_ERROR "Cannot find Root directory of Google Flags")
    SET(GFlags_FOUND false)
ENDIF(GFlags_LIBRARY_DIRS)
