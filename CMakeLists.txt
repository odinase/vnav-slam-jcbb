cmake_minimum_required(VERSION 3.0.2)
project(slam)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  )

find_package(Boost 1.50 REQUIRED)
  
find_package(
  catkin_simple REQUIRED
  gtsam_catkin
  eigen_catkin
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slam
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  eigen3
  jcbb/include
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_simple()

cs_add_library(jcbb
  src/jcbb/jcbb.cpp
  src/jcbb/MarginalMocks.cpp
  src/jcbb/Hypothesis.cpp
  src/jcbb/utils.cpp
)

target_link_libraries(jcbb
  ${catkin_LIBRARIES}
)

cs_add_library(slam
  src/slam/slam.cpp
)

target_link_libraries(slam
  jcbb
  ${catkin_LIBRARIES}
)

cs_add_executable(slam_node
  src/slam/slam_node
)

target_link_libraries(slam_node
  ${catkin_LIBRARIES}
  slam
)