cmake_minimum_required(VERSION 2.8.3)
project(imu_velodyne_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  pcl_ros
  sensor_msgs
  pcl_conversions
  message_generation
)

#add_message_files
#(FILES )

#add_service_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)


add_executable(subtract_plane nodes/subtract_plane.cpp)
target_link_libraries(subtract_plane ${catkin_LIBRARIES})

