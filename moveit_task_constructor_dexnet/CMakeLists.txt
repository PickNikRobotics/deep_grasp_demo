cmake_minimum_required(VERSION 3.0.2)
project(gqcnn_demo)

add_compile_options(-std=c++14)

add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)


find_package(catkin REQUIRED COMPONENTS
	actionlib
	cv_bridge
	geometry_msgs
	image_transport
	message_generation
	moveit_core
	moveit_ros_planning_interface
	moveit_task_constructor_core
	moveit_task_constructor_msgs
	roscpp
	rosparam_shortcuts
	rospy
	sensor_msgs
	std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.2 REQUIRED)

add_service_files(
  FILES
  GQCNNGrasp.srv
  Images.srv
)


generate_messages(
  DEPENDENCIES
	geometry_msgs
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
 	geometry_msgs
	message_runtime
	moveit_task_constructor_msgs
 	roscpp
	rospy
	sensor_msgs
	std_msgs
 DEPENDS
  EIGEN3
 	OpenCV
)


include_directories(
	SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)


add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/image_server.cpp
)


target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(gqcnn_action_server
               src/gqcnn_action_server.cpp
)

target_link_libraries(gqcnn_action_server
  ${catkin_LIBRARIES}
	${PROJECT_NAME}
  ${Eigen3_LIBRARIES}
)


add_executable(gqcnn_pick_place_demo
               src/gqcnn_pick_place_demo.cpp
							 src/gqcnn_pick_place_task.cpp
)

target_link_libraries(gqcnn_pick_place_demo
                      ${catkin_LIBRARIES}
)


catkin_install_python(PROGRAMS
  src/gqcnn_server
	scripts/grasp_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS gqcnn_action_server gqcnn_pick_place_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
