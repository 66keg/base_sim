cmake_minimum_required(VERSION 2.8.3)
project(base_sim)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_srvs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_sim_node src/base_sim_node.cpp)
target_link_libraries(base_sim_node ${catkin_LIBRARIES})
