# Author: Luke Dennis l.dennis@unsw.edu.au

cmake_minimum_required(VERSION 3.0.2)
project(lab04_example)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  genmsg # Needed for generating srv message
  roslaunch
)

# Perform checks on launch files too
roslaunch_add_file_check(launch)

add_service_files(FILES KeyToCmdVel.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_key_input src/key_input.cpp)
target_link_libraries(${PROJECT_NAME}_key_input  ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_vel_cmd_node src/velocity_command_node.cpp)
target_link_libraries(${PROJECT_NAME}_vel_cmd_node  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_vel_cmd_node ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME}_vel_server src/key_to_cmd_vel_server.cpp)
target_link_libraries(${PROJECT_NAME}_vel_server ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_vel_server ${PROJECT_NAME}_gencpp)

