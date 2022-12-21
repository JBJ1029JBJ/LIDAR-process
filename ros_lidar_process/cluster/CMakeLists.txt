cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++14)

project(cluster)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  laser_geometry
  pcl_ros
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES targetless
    CATKIN_DEPENDS cv_bridge image_geometry pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)

add_executable(
${PROJECT_NAME}
${project_HEADERS}
#src/main.cpp 
#src/fusion.cpp
src/cluster.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
