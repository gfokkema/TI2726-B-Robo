cmake_minimum_required(VERSION 2.8.3)
project(linefollower)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
catkin_package(
)

add_executable(analyzer src/analyzer.cpp)
target_link_libraries(analyzer ${catkin_LIBRARIES})

