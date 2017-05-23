cmake_minimum_required(VERSION 2.8.3)
project(pcd_visualizer)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslaunch
        pcl_ros
        pcl_conversions
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

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
SET (libvtk /usr/lib/libvtkFiltering.so
            /usr/lib/libvtkGraphics.so)

add_executable(viewer visualizer.cpp)
target_link_libraries(viewer ${catkin_LIBRARIES} ${libvtk})
