cmake_minimum_required(VERSION 3.0.2)
project(lab04_example)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_key_input src/key_input.cpp)
target_link_libraries(${PROJECT_NAME}_key_input  ${catkin_LIBRARIES} )

