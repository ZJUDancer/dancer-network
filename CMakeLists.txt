cmake_minimum_required(VERSION 2.8.3)
project(dnetwork)

add_definitions(-std=c++11 -Wall -O2)

find_package(catkin
  REQUIRED
  dmsgs
  dtransmit
  dmotion
  dbehavior
  dvision
  dprocess
  roscpp
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dnetwork
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/team.cpp
  src/gamecontroller.cpp
  )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
