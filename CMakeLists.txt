cmake_minimum_required(VERSION 2.8.3)
project(velocity_command_to_twist)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs bupimo_msgs roscpp)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(
  velocity_command_to_twist
  src/velocity_command_to_twist.cpp 
)	 

target_link_libraries(velocity_command_to_twist ${catkin_LIBRARIES})

