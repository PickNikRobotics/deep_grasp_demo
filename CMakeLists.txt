cmake_minimum_required(VERSION 3.0.2)
project(deep_grasp_demo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	moveit_core
	moveit_ros_planning_interface
	moveit_task_constructor_core
	moveit_task_constructor_msgs
	roscpp
	rosparam_shortcuts
)

catkin_package(
	CATKIN_DEPENDS
		moveit_task_constructor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gpd_action_server src/gpd_action_server.cpp )
target_link_libraries(gpd_action_server ${catkin_LIBRARIES})

add_executable(gpd_pick_place_demo src/gpd_pick_place_demo.cpp src/gpd_pick_place_task.cpp)
target_link_libraries(gpd_pick_place_demo ${catkin_LIBRARIES})

install(TARGETS gpd_action_server gpd_pick_place_demo  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
