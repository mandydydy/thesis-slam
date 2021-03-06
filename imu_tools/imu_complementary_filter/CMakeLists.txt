cmake_minimum_required(VERSION 2.8.3)
project(imu_complementary_filter)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES complementary_filter
  CATKIN_DEPENDS message_filters roscpp sensor_msgs std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(complementary_filter
  src/complementary_filter.cpp
  src/complementary_filter_ros.cpp
  include/imu_complementary_filter/complementary_filter.h
  include/imu_complementary_filter/complementary_filter_ros.h
)


# create complementary_filter_node executable
add_executable(complementary_filter_ros
  src/complementary_filter_ros.cpp)
target_link_libraries(complementary_filter_ros complementary_filter ${catkin_LIBRARIES})

install(TARGETS complementary_filter complementary_filter_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
