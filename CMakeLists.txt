cmake_minimum_required(VERSION 2.8.3)
project(gakachu_backend)

find_package(
	catkin REQUIRED COMPONENTS
  	rospy
)

catkin_python_setup()

catkin_package()

catkin_install_python(
	PROGRAMS
        src/main.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/start_server.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
