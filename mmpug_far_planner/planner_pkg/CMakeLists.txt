cmake_minimum_required(VERSION 3.0.2)
project(planner_pkg)

## Compile as C++11
# add_compile_options(-std=c++14)
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  mmpug_msgs
  tf2
  tf2_geometry_msgs
  pcl_conversions
  cv_bridge
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



################################################
## Declare ROS messages, services and actions ##
################################################

# Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   RobotPlan.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs   
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES planner_pkg
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(grid_planner 
src/planner_ros.cpp 
src/grid_planner.cpp
src/grid_planner_astar.cpp
src/grid_planner_thetastar.cpp
)

add_dependencies(grid_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(grid_planner
    ${catkin_LIBRARIES}
)
