cmake_minimum_required(VERSION 3.0.2)
project(reading_pcd)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy 
  sensor_msgs 
  geometry_msgs 
)
find_package(PCL REQUIRED)
generate_messages( 
  DEPENDENCIES 
  sensor_msgs 
  geometry_msgs
) 

catkin_package(
CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs
) 

include_directories( 
${catkin_INCLUDE_DIRS} 
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
