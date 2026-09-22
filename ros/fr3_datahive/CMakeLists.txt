cmake_minimum_required(VERSION 3.0.2)
project(fr3_datahive)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
catkin_install_python(PROGRAMS scripts/record.py scripts/uvc_camera.py
  scripts/collision_behavior.py scripts/home_position.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
