cmake_minimum_required(VERSION 3.0.2)
project(buoy_projector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cortix_msgs
  cv_bridge
  image_transport
  mdt_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  camera_info_manager
  image_geometry
)

#
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES buoy_projector
  DEPENDS system_lib
)

#catkin_python_setup()

catkin_install_python(PROGRAMS
  src/projector.py
  src/marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cam_info src/cam_info.cpp)
target_link_libraries(cam_info ${catkin_LIBRARIES})

install(TARGETS cam_info
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  launch
  include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

#install(TARGETS buoy_projector
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

