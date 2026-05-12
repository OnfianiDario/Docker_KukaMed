cmake_minimum_required(VERSION 3.10)
project(kuka_ros_node)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../include   # include/ della repo
)

catkin_package()

  add_executable(med_driver
    src/med_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/TcpServer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/Controller.cpp
  )

  add_executable(demo_node
    src/demo_node.cpp
  )

  target_link_libraries(med_driver ${catkin_LIBRARIES})
  target_link_libraries(demo_node ${catkin_LIBRARIES})