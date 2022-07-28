include_directories(${CMAKE_CURRENT_BINARY_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
add_library (libjpeg jpeg-data.c jpeg-marker.c)
SET_TARGET_PROPERTIES( libjpeg PROPERTIES COMPILE_FLAGS "-fPIC")
