cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0054 NEW)


#add_definitions(-std=c++11 -Wall -Wextra)
project(droneport_sitl_gazebo VERSION 1.0.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
  set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Build type" FORCE)
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage;AddressSanitizer;UndefinedBehaviorSanitizer")
message(STATUS "cmake build type: ${CMAKE_BUILD_TYPE}")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Set c++11 or higher
include(EnableC++XX)
# Set c11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(ccache)
include(GNUInstallDirs)

## FIND DEPENDENCIES ##
find_package(Boost 1.40 REQUIRED COMPONENTS system thread filesystem)
find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)

pkg_check_modules(OGRE OGRE)

find_package(Threads)

#set(PROTOBUF_IMPORT_DIRS "")
#foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#  if(ITR MATCHES ".*gazebo-[0-9.]+$")
#    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
#  endif()
#endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
#find_package(Protobuf REQUIRED)
#pkg_check_modules(PROTOBUF protobuf)


#if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
#  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
#endif()

if("${GAZEBO_VERSION}" VERSION_LESS "9.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message(STATUS "Gazebo version: ${GAZEBO_VERSION}")
endif()

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})


find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# find MAVLink
find_package(MAVLink)
find_package(OpenCV REQUIRED)
# find Gstreamer
#set(GStreamer_FIND_VERSION "1.0")
#  find_package(GStreamer REQUIRED)

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

## BUILD ##

add_compile_options(-Wno-deprecated-declarations -Wno-address-of-packed-member)

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
    include 
    ${Boost_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
    ${GAZEBO_INCLUDE_DIRS}
    ${GAZEBO_MSG_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}/Paging		# Workaround for "fatal error: OgrePagedWorldSection.h: No such file or directory"    
    ${MAVLINK_INCLUDE_DIRS}    
    ${OpenCV_INCLUDE_DIRS}    
)

#if (GSTREAMER_FOUND)
#  include_directories(
#    ${GSTREAMER_INCLUDE_DIRS}
#  )
#endif()

link_libraries(
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBRARIES}  
  )

#if (GSTREAMER_FOUND)
#  message(${GSTREAMER_LIBRARIES})
#  link_libraries(
#    ${GSTREAMER_LIBRARIES}
#    ${GLIB_LDFLAGS}
#    gobject-2.0
#  )
#
#  if (APPLE)
#    link_libraries(
#      ${ICU_UC_LDFLAGS}
#    )
#  endif()
#endif()

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_LIBRARY_DIRS}  
  )


#set(std_msgs msgs/Int32.proto)
#set(sensor_msgs
 # msgs/SITLGps.proto
 # msgs/Int32.proto
 # )

#PROTOBUF_GENERATE_CPP(STD_PROTO_SRCS STD_PROTO_HDRS ${std_msgs})
#PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

#add_library(std_msgs SHARED ${STD_PROTO_SRCS})
#add_library(sensor_msgs SHARED ${SEN_PROTO_SRCS})

#add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)
add_library(gazebo_droneport_plugin SHARED src/gazebo_droneport_plugin.cpp src/droneport_mavlink_interface.cpp)
#add_library(gazebo_droneport_plugin SHARED src/gazebo_droneport_plugin.cpp thirdparty/PX4-SITL_gazebo/src/mavlink_interface.cpp)
#add_library(gazebo_gst_camera_plugin SHARED src/gazebo_gst_camera_plugin.cpp)

set(plugins
  gazebo_droneport_plugin
  #gazebo_gps_plugin
  )

foreach(plugin ${plugins})
  target_include_directories(${plugin} PUBLIC  ~/PX4-Autopilot/build/px4_sitl_default/build_gazebo/)
  target_link_libraries(${plugin} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} Threads::Threads ${PROTOBUF_LIBRARY} ~/PX4-Autopilot/build/px4_sitl_default/build_gazebo/libsensor_msgs.so)
endforeach()

#if (GSTREAMER_FOUND)
#  add_library(gazebo_gst_camera_plugin SHARED src/gazebo_gst_camera_plugin.cpp)
#  set(plugins
#    ${plugins}
#    gazebo_gst_camera_plugin
#  )
#  message(STATUS "Found GStreamer: adding gst_camera_plugin")
  # if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
  #   QT4_WRAP_CPP(headers_MOC include/gazebo_video_stream_widget.h)
  #   add_library(gazebo_video_stream_widget SHARED ${headers_MOC} src/gazebo_video_stream_widget.cpp)
  #   target_link_libraries(gazebo_video_stream_widget ${GAZEBO_LIBRARIES} ${QT_LIBRARIES})
  #   set(pluginstranspa
  #     ${plugins}
  #     gazebo_video_stream_widget
  #   )
  #   message(STATUS "Found GStreamer: adding gst_video_stream_widget")
  # else()
  #   QT5_WRAP_CPP(headers_MOC include/gazebo_video_stream_widget.h)
  #   add_library(gazebo_video_stream_widget SHARED ${headers_MOC} src/gazebo_video_stream_widget.cpp)
  #   target_link_libraries(gazebo_video_stream_widget ${GAZEBO_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Qt5Test_LIBRARIES})
  #   set(plugins
  #     ${plugins}
  #     gazebo_video_stream_widget
  #   )
  #   message(STATUS "Found GStreamer: adding gst_video_stream_widget")
  # endif()
#endif()

#foreach(plugin ${plugins})
#  add_dependencies(${plugin} sensor_msgs)
#endforeach()

#link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_library(gazebo_droneport_plugin SHARED gazebo_droneport_plugin.cc)
#add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)

#target_link_libraries(gazebo_droneport_plugin ${GAZEBO_LIBRARIES})


