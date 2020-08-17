cmake_minimum_required(VERSION 2.8.3)
project(flocking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  message_generation
  message_filters
)

add_message_files(DIRECTORY msg FILES Neighbors.msg ModeStamped.msg)

generate_messages(
   DEPENDENCIES
   std_msgs 
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Formation SensorNeighbor CommandSender MathUtils
  CATKIN_DEPENDS roscpp std_msgs nav_msgs mrs_msgs mrs_lib message_runtime message_filters
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(MathUtils src/MathUtils.cpp)

add_library(SensorNeighbor src/SensorNeighbor.cpp)
add_dependencies(SensorNeighbor flocking_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_library(Formation src/Formation.cpp)
add_dependencies(Formation flocking_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_library(CommandSender src/CommandSender.cpp)
add_dependencies(CommandSender ${catkin_EXPORTED_TARGETS})

target_link_libraries(SensorNeighbor
  MathUtils
  ${catkin_LIBRARIES}
)

target_link_libraries(Formation
  MathUtils
  ${catkin_LIBRARIES}
)

target_link_libraries(CommandSender
  ${catkin_LIBRARIES}
)
