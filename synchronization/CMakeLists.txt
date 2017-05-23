cmake_minimum_required(VERSION 2.8.3)
project(synchronization)

find_package(catkin REQUIRED
        roscpp
        roslaunch
        sensor_msgs
        geometry_msgs
        tf
        )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_tf
  CATKIN_DEPENDS  roscpp roslaunch sensor_msgs geometry_msgs tf
#  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

#add_dependencies(imu_syn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(imu_syn src/imu_syn.cpp)
target_link_libraries(imu_syn ${catkin_LIBRARIES})