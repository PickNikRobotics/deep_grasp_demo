cmake_minimum_required(VERSION 3.0.2)
project(gqcnn_demo)

add_compile_options(-std=c++14)

add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	image_transport
	roscpp
	rosparam_shortcuts
	rospy
	sensor_msgs
	std_msgs
	std_srvs
)

find_package(OpenCV 3.2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gqcnn_demo
 CATKIN_DEPENDS
 	roscpp
	rospy
	sensor_msgs
	std_msgs
 DEPENDS
 	OpenCV
)

include_directories(
	SYSTEM
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gqcnn_action_server src/gqcnn_action_server.cpp)
target_link_libraries(gqcnn_action_server
  ${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS gqcnn_action_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
