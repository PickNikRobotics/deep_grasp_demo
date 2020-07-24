cmake_minimum_required(VERSION 3.0.2)
project(deep_grasp_demo)

add_compile_options(-std=c++14)

add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)


find_package(catkin REQUIRED COMPONENTS
	actionlib
	geometry_msgs
	gpd_ros
	moveit_core
	moveit_ros_planning_interface
	moveit_task_constructor_core
	moveit_task_constructor_msgs
	pcl_conversions
	roscpp
	rosparam_shortcuts
	sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
	CATKIN_DEPENDS
		geometry_msgs
		moveit_task_constructor_msgs
		pcl_conversions
		sensor_msgs
 DEPENDS
 		EIGEN3
		PCL
)


include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gpd_action_server src/gpd_action_server.cpp )
target_link_libraries(gpd_action_server
                      ${catkin_LIBRARIES}
											${gpd_ros_LIBRARIES}
											${Eigen3_LIBRARIES}
											${PCL_LIBRARIES}
)

add_executable(gpd_pick_place_demo src/gpd_pick_place_demo.cpp src/gpd_pick_place_task.cpp)
target_link_libraries(gpd_pick_place_demo
                      ${catkin_LIBRARIES}
)

install(TARGETS gpd_action_server gpd_pick_place_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
