cmake_minimum_required(VERSION 3.1.0)
project(teleop_twist_rewrite)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)
catkin_package()

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
