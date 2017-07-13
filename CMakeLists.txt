cmake_minimum_required(VERSION 2.8.3)
project(dnetwork)

add_definitions(-std=c++11 -Wall -O2)

find_package(catkin
  REQUIRED
  message_generation
  dtransmit
  dmotion
  dbehavior
  dvision
  dprocess
  roscpp
  )

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   GCInfo.msg
   TeamInfo.msg
 )


# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
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

  )

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

