cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ros_status_monitor)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    ros_type_introspection
    topic_tools
)

catkin_package(
  CATKIN_DEPENDS roscpp ros_type_introspection topic_tools
  )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

include_directories ("${PROJECT_SOURCE_DIR}/include")

add_executable( status_monitor_node
  src/status_monitor_node.cpp
)
target_link_libraries( status_monitor_node
  ${catkin_LIBRARIES}
)