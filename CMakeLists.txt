cmake_minimum_required(VERSION 3.5)
project(arm_camera_calibration)

add_compile_options(-std=c++11)

ADD_DEFINITIONS(-DDEBUG)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  visp_bridge
  tf
  orocos_kdl
  kdl_parser
  urdf
  roslib
  roslint
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS

)


find_package(VISP REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)

include_directories(
    src
    include
    ${catkin_INCLUDE_DIRS}
    ${VISP_INCLUDE_DIRS}
)

add_executable(robot_node src/robot_node.cpp)
target_link_libraries(robot_node ${catkin_LIBRARIES} ${VISP_LIBRARIES})
add_dependencies(robot_node ${catkin_EXPORTED_TARGETS})