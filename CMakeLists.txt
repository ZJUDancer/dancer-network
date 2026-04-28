cmake_minimum_required(VERSION 2.8.3)
project(dnetwork)

add_definitions(-std=c++11 -Wall -O2)

set(dmsgs_DIR /home/nvidia/robocup_ws/lib/devel/share/dmsgs/cmake)
set(dtransmit_DIR /home/nvidia/robocup_ws/lib/devel/share/dtransmit/cmake)
set(dprocess_DIR /home/nvidia/robocup_ws/lib/devel/share/dprocess/cmake)

find_package(catkin
  REQUIRED
  roscpp
  std_msgs
  dmsgs
  dconfig
  dtransmit
  dprocess
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED alsa)
find_library(FFTW3F_LIB NAMES fftw3f)
if(NOT FFTW3F_LIB)
  message(FATAL_ERROR "FFTW3 float library is required: install libfftw3-dev")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dnetwork
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ALSA_INCLUDE_DIRS}
  src/whistle_detection/src
  src/whistle_detection/third_party/include
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/team.cpp
  src/gamecontroller.cpp
  )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(whistle_detection_node
  src/whistle_detection/src/main.cpp
  src/whistle_detection/src/whistle_detection.cpp
  src/whistle_detection/third_party/src/duerwen_alsa.c
  )

target_link_libraries(whistle_detection_node
  ${ALSA_LIBRARIES}
  ${FFTW3F_LIB}
  pthread
  ${catkin_LIBRARIES}
  )
