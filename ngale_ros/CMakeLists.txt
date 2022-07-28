cmake_minimum_required(VERSION 2.8)
project(ngale_ros)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
add_definitions(-DCUDA_USE_STATIC_CUDA_RUNTIME=OFF)
# Find CUDA
FIND_PACKAGE(CUDA)
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
set(
CUDA_NVCC_FLAGS
${CUDA_NVCC_FLAGS};
-O3
-gencode arch=compute_30,code=sm_30
-gencode arch=compute_35,code=sm_35
-gencode arch=compute_50,code=[sm_50,compute_50]
-gencode arch=compute_52,code=[sm_52,compute_52]
-gencode arch=compute_61,code=sm_61
-gencode arch=compute_62,code=sm_62
)
add_definitions(-DGPU)
link_directories(
/usr/local/cuda/include
/usr/local/cuda/lib64
)

# exifyay
include_directories(exifyay)
#add_subdirectory (exifyay/libexif) 
set (GETTEXT_PACKAGE "libexif-12")
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/exifyay/libexif/config.h.cmake
               ${CMAKE_CURRENT_SOURCE_DIR}/exifyay/libexif/config.h)
add_library (libexif 
    exifyay/libexif/canon/exif-mnote-data-canon.c
    exifyay/libexif/canon/mnote-canon-entry.c
    exifyay/libexif/canon/mnote-canon-tag.c
    exifyay/libexif/exif-byte-order.c
    exifyay/libexif/exif-content.c
    exifyay/libexif/exif-data.c
    exifyay/libexif/exif-entry.c
    exifyay/libexif/exif-format.c
    exifyay/libexif/exif-ifd.c
    exifyay/libexif/exif-loader.c
    exifyay/libexif/exif-log.c
    exifyay/libexif/exif-mem.c
    exifyay/libexif/exif-mnote-data.c
    exifyay/libexif/exif-tag.c
    exifyay/libexif/exif-utils.c
    exifyay/libexif/fuji/exif-mnote-data-fuji.c
    exifyay/libexif/fuji/mnote-fuji-entry.c
    exifyay/libexif/fuji/mnote-fuji-tag.c
    exifyay/libexif/olympus/exif-mnote-data-olympus.c
    exifyay/libexif/olympus/mnote-olympus-entry.c
    exifyay/libexif/olympus/mnote-olympus-tag.c
    exifyay/libexif/pentax/exif-mnote-data-pentax.c
    exifyay/libexif/pentax/mnote-pentax-entry.c
    exifyay/libexif/pentax/mnote-pentax-tag.c)
SET_TARGET_PROPERTIES(libexif PROPERTIES COMPILE_FLAGS "-fPIC")
target_link_libraries(libexif m)

#add_subdirectory (exifyay/libjpeg) 
add_library (libjpeg exifyay/libjpeg/jpeg-data.c exifyay/libjpeg/jpeg-marker.c)
SET_TARGET_PROPERTIES( libjpeg PROPERTIES COMPILE_FLAGS "-fPIC")

#add_subdirectory (exifyay/JpegEncoderEXIF)
add_library (JpegEncoderEXIF exifyay/JpegEncoderEXIF/JpegEncoderEXIF.c)
SET_TARGET_PROPERTIES(JpegEncoderEXIF PROPERTIES COMPILE_FLAGS "-fPIC")

add_executable(test-exif src/test-exif.c)
target_link_libraries(test-exif m libexif libjpeg JpegEncoderEXIF)

#add_executable(misbst0601 src/misbst0601.c)
#target_link_libraries(test-exif m libexif libjpeg JpegEncoderEXIF)
#add wrapper
set(TRT_WRAPPER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_subdirectory(${TRT_WRAPPER_DIR})
include_directories(${TRT_WRAPPER_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

#add opencv
FIND_PACKAGE(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARIES_DIRS})

#add gstreamer
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.8 gstreamer-app-1.0>=1.8)
pkg_check_modules (LIBGSTREAMER10 libgstreamer10)
include_directories(${GST_INCLUDE_DIRS})
link_libraries(${GST_LIBRARIES})

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    birdseye_msgs
    angles
)

# Message and service files
add_message_files(
   FILES
   param.msg
 )

add_service_files(
  FILES
  start_service.srv
  stop_service.srv
 ) 

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    message_runtime
)


include_directories(
  include
  /usr/local/include/
  ${catkin_INCLUDE_DIRS}
)

#build ngale_exe
add_executable(ngale_exe src/main.cpp src/detector.cpp src/misbst0601.c src/parse_yaml.cpp src/utils.cpp)
#add_executable(ngale_exe src/main.cpp src/detector.cpp src/parse_yaml.cpp src/utils.cpp)
add_dependencies(ngale_exe ngale_ros_gencpp birdseye_msgs_generate_messages_cpp libexif libjpeg JpegEncoderEXIF)
target_link_libraries(ngale_exe
  m
  libexif libjpeg JpegEncoderEXIF
  pthread
  stdc++
  cuda
  cudart
  cublas
  curand
  gstnet-1.0
  gstcheck-1.0
  gstcontroller-1.0
  gstreamer-1.0
  gstbase-1.0
  gstapp-1.0
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  TrtNet
  yaml-cpp
)

# RGB message reader
add_executable(listen_rgb src/listener_rgb.cpp)
target_link_libraries(listen_rgb ${catkin_LIBRARIES} )

# RGB message reader
add_executable(listen_thermal src/listener_thermal.cpp)
target_link_libraries(listen_thermal ${catkin_LIBRARIES} )

# Start service
add_executable(start_client src/start_client.cpp)
target_link_libraries(start_client ${catkin_LIBRARIES} )
add_dependencies(start_client ngale_ros_gencpp ngale_ros_generated_messages_cpp)

# Start service
add_executable(stop_client src/stop_client.cpp)
target_link_libraries(stop_client ${catkin_LIBRARIES} )
add_dependencies(stop_client ngale_ros_gencpp ngale_ros_generated_messages_cpp)


# Mark executables and/or libraries for installation
install(TARGETS
  ngale_exe
  listen_rgb
  listen_thermal
  start_client
  stop_client
  libjpeg libexif JpegEncoderEXIF
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark cpp header files for installation
install(DIRECTORY
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY
  cfg
  models
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # FILES_MATCHING PATTERN "*.yaml"
  PATTERN ".svn" EXCLUDE
)

# install(FILES
#   launch/rosconsole.config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

