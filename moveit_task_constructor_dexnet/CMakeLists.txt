cmake_minimum_required(VERSION 3.0.2)
project(moveit_task_constructor_dexnet)

# C++ 14
add_compile_options(-std=c++14)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	actionlib
	cv_bridge
	geometry_msgs
	image_transport
	message_generation
	moveit_task_constructor_msgs
	roscpp
	rosparam_shortcuts
	rospy
	sensor_msgs
	std_msgs
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.2 REQUIRED)

# Service files
add_service_files(
  FILES
  GQCNNGrasp.srv
  Images.srv
)

# Message generation dependencies
generate_messages(
  DEPENDENCIES
	geometry_msgs
  std_msgs
)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
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

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
	SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(grasp_image_detection
							 src/grasp_detection.cpp
               src/grasp_image_detection.cpp
)

add_executable(process_image_server
							 src/image_server.cpp
							 src/process_image_server.cpp
)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(grasp_image_detection
								 ${${PROJECT_NAME}_EXPORTED_TARGETS}
								 ${catkin_EXPORTED_TARGETS}
)

add_dependencies(process_image_server
								 ${${PROJECT_NAME}_EXPORTED_TARGETS}
								 ${catkin_EXPORTED_TARGETS}
)

# Specify libraries to link a library or executable target against
target_link_libraries(grasp_image_detection
										  ${catkin_LIBRARIES}
										  ${Eigen3_LIBRARIES}
)

target_link_libraries(process_image_server
                      ${catkin_LIBRARIES}
											${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# Mark python scripts for install
catkin_install_python(PROGRAMS
  src/gqcnn_server
	scripts/grasp_detector
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables and/or libraries for installation
install(
  TARGETS
    grasp_image_detection process_image_server
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Mark roslaunch files for installation
install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark config files for installation
install(
  DIRECTORY
    config
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
