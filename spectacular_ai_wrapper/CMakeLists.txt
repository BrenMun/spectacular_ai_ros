cmake_minimum_required(VERSION 3.0.2)
project(spectacular_ai_wrapper)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy
                 sensor_msgs
                 std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

##################################################
#              PYTHON EXECUTABLES                #
##################################################
catkin_install_python(PROGRAMS
  scripts/oakd_spectacularai_node.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)