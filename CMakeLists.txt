cmake_minimum_required(VERSION 2.8.3)
project(cal_overlap_ratio)

set(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp  std_msgs
  DEPENDS EIGEN3 PCL

)

add_executable(ply2pcd src/ply2pcd.cpp)
target_link_libraries(ply2pcd  ${PCL_LIBRARIES} )

add_executable(gen_overlap_clouds src/gen_overlap_clouds.cpp)
target_link_libraries(gen_overlap_clouds   ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#set(GROUND_REMOVAL src/removal_func.cpp src/gaus_blur.cpp src/ground_removal.cpp src/ground_remove_RANSAC.cpp)
add_executable(cal_overlap_ratio src/overlap_ratio.cpp ${GROUND_REMOVAL})
target_link_libraries(cal_overlap_ratio ${catkin_LIBRARIES} ${PCL_LIBRARIES} )




