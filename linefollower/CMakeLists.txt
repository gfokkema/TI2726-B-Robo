cmake_minimum_required(VERSION 3.1.0)
project(linefollower)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
)
catkin_package()

add_executable(analyzer src/analyzer.cpp)
target_link_libraries(analyzer ${catkin_LIBRARIES})
