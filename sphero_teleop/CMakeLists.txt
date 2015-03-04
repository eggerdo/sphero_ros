cmake_minimum_required(VERSION 2.8.3)
project(sphero_teleop)

find_package(catkin REQUIRED geometry_msgs roscpp)

catkin_package(DEPENDS geometry_msgs roscpp)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sphero_teleop_joy
  src/sphero_joy.cpp
)
target_link_libraries(sphero_teleop_joy
	${catkin_LIBRARIES}
)
    
install(TARGETS sphero_teleop_joy
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
