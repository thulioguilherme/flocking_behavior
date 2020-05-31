cmake_minimum_required(VERSION 2.8.3)
project(flocking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  nav_msgs
  mrs_msgs
  mrs_lib
  message_generation
  std_msgs
)

add_message_files(
   FILES
   Neighbors.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs 
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Formation SensorNeighbor CommandSender MathOperations
  CATKIN_DEPENDS roscpp std_msgs nav_msgs mrs_lib mrs_msgs message_runtime
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(MathOperations src/MathOperations.cpp)

add_library(SensorNeighbor src/SensorNeighbor.cpp)
add_dependencies(SensorNeighbor ${catkin_EXPORTED_TARGETS})

add_library(Formation src/Formation.cpp)
add_dependencies(Formation ${catkin_EXPORTED_TARGETS})

add_library(CommandSender src/CommandSender.cpp)
add_dependencies(CommandSender ${catkin_EXPORTED_TARGETS})


target_link_libraries(SensorNeighbor
  MathOperations
  ${catkin_LIBRARIES}
)

target_link_libraries(Formation
  ${catkin_LIBRARIES}
)

target_link_libraries(CommandSender
  ${catkin_LIBRARIES}
)