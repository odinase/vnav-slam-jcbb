cmake_minimum_required(VERSION 3.10)
project(slam)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  message_filters
  tf2
  tf2_ros
  )

# find_package(Boost 1.50 REQUIRED)
  
# find_package(
#   catkin_simple REQUIRED
#   # gtsam_catkin
#   # eigen_catkin
# )

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(Boost REQUIRED)

find_package (glog 0.6.0 REQUIRED)

find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES slam
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs message_filters
#  DEPENDS system_lib
)

include_directories(
  eigen3
  jcbb/include
  include
  ${catkin_INCLUDE_DIRS}
)

# catkin_simple()

add_library(jcbb
  src/jcbb/jcbb.cpp
  src/jcbb/MarginalMocks.cpp
  src/jcbb/Hypothesis.cpp
  src/jcbb/utils.cpp
)

target_link_libraries(jcbb
  # ${catkin_LIBRARIES}
  Eigen3::Eigen
  gtsam
  )

add_library(slam
  src/slam/slam.cpp
)

target_link_libraries(slam
  jcbb
  # ${catkin_LIBRARIES}
  Eigen3::Eigen
  gtsam
)

add_executable(slam_node
  src/slam/slam_node
)

target_link_libraries(slam_node
  ${catkin_LIBRARIES}
  slam
  gtsam
  Eigen3::Eigen
  glog::glog
)