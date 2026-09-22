cmake_minimum_required(VERSION 3.0.2)
project(fr3_control)

set(CMAKE_BUILD_TYPE Release)
# 17, not serl's 20: libfranka >=0.18 forces C++17 on every consumer and the
# Dockerfile already patches franka_ros to match. Nothing here needs C++20.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  geometry_msgs
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  std_msgs
  std_srvs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
add_message_files(FILES
  ZeroJacobian.msg
  PoseError.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fr3_control
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    std_msgs
    std_srvs
  DEPENDS Franka
)

add_library(fr3_control
  src/cartesian_impedance_controller.cpp
)

add_dependencies(fr3_control
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(fr3_control PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(fr3_control SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(fr3_control PUBLIC
  include
)
install(TARGETS fr3_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES fr3_control_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
