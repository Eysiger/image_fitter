# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(image_fitter)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  message_generation
  grid_map_core
  grid_map_ros
  grid_map_cv
)

## Generate messages in the 'msg' folder
# add_message_files(
#  FILES
#  Int.msg
#  Float.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#  FILES
#  FirstStep.srv
#  Shutdown.srv
#   Starter.srv
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#)

# Declare this project as a catkin package
catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  roscpp
  sensor_msgs
  tf
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories(${PCL_INCLUDE_DIRS})

# Define an execuable target called hello_world_node 
add_executable(${PROJECT_NAME} src/image_fitter_node.cpp
            src/ImageFitter.cpp)

# Link the hello_world_node target against the libraries used by roscpp
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

add_dependencies(${PROJECT_NAME} image_fitter_gencpp)

