cmake_minimum_required(VERSION 3.0.2)
project(moveit_task_constructor_gpd)

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
  eigen_conversions
  geometry_msgs
  message_generation
  moveit_task_constructor_msgs
  roscpp
  rosparam_shortcuts
  sensor_msgs
  std_msgs
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
if (GPD_LIB)
  message(STATUS "Library GPD found in ${GPD_LIB}")
else()
  message(FATAL_ERROR "Library GPD not found")
endif()

find_path(GPD_INCLUDE_DIRS NAMES gpd PATHS /usr/local/include NO_DEFAULT_PATH)
if (GPD_INCLUDE_DIRS)
  message(STATUS "Include directory GPD found in ${GPD_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "Include directory GPD not found")
endif()

# Service files
add_service_files(
  FILES
  PointCloud.srv
)

# Message generation dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    message_runtime
    moveit_task_constructor_msgs
    sensor_msgs
    std_msgs
  DEPENDS
    EIGEN3
    PCL
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GPD_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library with project namespace to avoid naming collision
add_library(${PROJECT_NAME}
            src/${PROJECT_NAME}/cloud_utils.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Declare a C++ executable
add_executable(grasp_cloud_detection
  src/grasp_cloud_detection.cpp
  src/grasp_detection.cpp
)

add_executable(point_cloud_server
  src/cloud_server.cpp
  src/point_cloud_server.cpp
)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(grasp_cloud_detection
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(point_cloud_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


# Specify libraries to link a library or executable target against
target_link_libraries(grasp_cloud_detection
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GPD_LIB}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(point_cloud_server
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    grasp_cloud_detection ${PROJECT_NAME} point_cloud_server
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
