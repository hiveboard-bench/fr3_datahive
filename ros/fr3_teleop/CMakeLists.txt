cmake_minimum_required(VERSION 3.0.2)
project(fr3_teleop)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

catkin_install_python(PROGRAMS
  scripts/joystick_teleop.py
  scripts/fr3_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
