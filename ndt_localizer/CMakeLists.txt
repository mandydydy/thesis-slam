cmake_minimum_required(VERSION 2.8.3)
project(ndt_localizer)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  runtime_manager
  #velodyne_pointcloud
  cmake_modules
  message_generation
  imu_filter_madgwick
  message_filters
)

add_message_files(
FILES 
ndt_stat.msg
)

#add_service_files(
#  FILES
#  getImu.srv
#  getOdom.srv
#)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}

  CATKIN_DEPENDS runtime_manager
  CATKIN_DEPENDS imu_filter_madgwick
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
# G2O required
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

# For building the CHOLMOD / CSPARSE solvers
find_package(Cholmod)
include_directories(${CHOLMOD_INCLUDE_DIR})
find_package(BLAS)
find_package(LAPACK)
find_package(CSparse)
include_directories(${CSPARSE_INCLUDE_DIR})

SET ( SLAM_SRCS
    nodes/ndt_mapping/ndt_key_mapping.cpp
    nodes/graph_optimizer/optimizer.cpp
        )

SET ( SLAM2D_SRCS
        nodes/ndt_mapping/ndt_key_mapping.cpp
        nodes/graph_optimizer/optimizer2d.cpp
        )

SET (LIB_CXSPARSE /usr/lib/x86_64-linux-gnu/libcxsparse.so)
SET (SLAM_LIBS
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_SOLVER_CSPARSE_EXTENSION}
        ${G2O_TYPES_SLAM3D}
        ${G2O_TYPES_SLAM2D}
        )

#add_executable(ndt_mapping nodes/ndt_mapping/ndt_mapping.cpp)
#target_link_libraries(ndt_mapping ${catkin_LIBRARIES})
#add_dependencies(ndt_mapping runtime_manager_generate_messages_cpp)

#add_executable(ndt_key_mapping nodes/ndt_mapping/ndt_key_mapping.cpp)
#target_link_libraries(ndt_key_mapping ${catkin_LIBRARIES})
#add_dependencies(ndt_key_mapping runtime_manager_generate_messages_cpp)

add_executable(local2global nodes/local2global/local2global.cpp )
target_link_libraries(local2global ${catkin_LIBRARIES})
add_dependencies(local2global runtime_manager_generate_messages_cpp)

add_executable(slam_node nodes/slam_node.cpp ${SLAM_SRCS})
target_link_libraries(slam_node ${catkin_LIBRARIES} ${SLAM_LIBS} ${LIB_CXSPARSE})

#add_executable(slam2d_node nodes/slam_node.cpp ${SLAM2D_SRCS})
#target_link_libraries(slam2d_node ${catkin_LIBRARIES} ${SLAM_LIBS} ${LIB_CXSPARSE})
#add_library(slam_lib ${SLAM_SRCS})
#add_executable(slam_node nodes/slam_node.cpp)
#target_link_libraries(slam_node slam_lib ${catkin_LIBRARIES} ${SLAM_LIBS})
#add_dependencies(slam_node runtime_manager_generate_messages_cpp)

#add_executable(sync_test nodes/ndt_mapping/sync_test.cpp)
#target_link_libraries(sync_test ${catkin_LIBRARIES})
#add_dependencies(sync_test runtime_manager_generate_messages_cpp)


#add_executable(optimizer nodes/graph_optimizer/optimizer.cpp)
#target_link_libraries(optimizer ${catkin_LIBRARIES})
#add_dependencies(optimizer runtime_manager_generate_messages_cpp)

#add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
#target_link_libraries(ndt_matching ${catkin_LIBRARIES})
#add_dependencies(ndt_matching runtime_manager_generate_messages_cpp ndt_localizer_generate_messages_cpp)


