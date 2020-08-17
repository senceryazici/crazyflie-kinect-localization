cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_kinect_localization)

add_definitions(-std=c++11)
add_definitions(${PCL_DEFINITIONS})

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  pcl_ros
  sensor_msgs
  std_msgs
  message_generation
  message_runtime
  kinect2_bridge
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    pcl_ros
    tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Color Filter Node
add_executable(get_centroid src/get_centroid.cpp)
target_link_libraries(get_centroid ${PCL_LIBRARIES})
target_link_libraries(get_centroid ${catkin_LIBRARIES})

# Color Filter Node
add_executable(color_filter src/color_filter.cpp)
target_link_libraries(color_filter ${PCL_LIBRARIES})
target_link_libraries(color_filter ${catkin_LIBRARIES})

# CropBox Filter Node
add_executable(cropbox_filter src/cropbox_filter.cpp)
target_link_libraries(cropbox_filter ${PCL_LIBRARIES})
target_link_libraries(cropbox_filter ${catkin_LIBRARIES})

