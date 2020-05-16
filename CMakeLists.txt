cmake_minimum_required(VERSION 2.8.3)
project(pcl_downsampling_ws)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  pcl_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES PCL_study
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(downsampling src/downsampling.cpp) 

target_link_libraries(downsampling ${catkin_LIBRARIES})
