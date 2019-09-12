cmake_minimum_required(VERSION 2.8.3)
project(surface_reconstruction)


find_package(OpenCV REQUIRED)
find_package(PCL 1.6 REQUIRED)
 

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)


catkin_package(
 
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs 
 
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

 
add_executable(
reconstruction_node 

src/main.cpp
src/transformation.cpp
)


target_link_libraries(
reconstruction_node  

${catkin_LIBRARIES}
${OpenCV_LIBS}
${PCL_LIBRARIES}
)

 
