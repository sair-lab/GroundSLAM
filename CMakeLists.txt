cmake_minimum_required(VERSION 3.5.1)
project(ni_slam)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  # eigen_conversions
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

find_package(OpenCV 4.2 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(FFTW3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(VTK REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -Wno-reorder")
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES correlation_flow
 CATKIN_DEPENDS geometry_msgs visualization_msgs image_transport nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib SHARED
  src/optimization_2d/pose_graph_2d.cc
  src/utils.cc
  src/dataset.cc
  src/camera.cc
  src/frame.cc
  src/edge.cc
  src/correlation_flow.cc
  src/loop_closure.cc
  src/map.cc
  src/map_builder.cc
  src/thread_publisher.cc
  src/map_stitcher.cc
  src/visualization.cc
  src/timer.cc
)

target_link_libraries(${PROJECT_NAME}_lib
  -lfftw3 
  -lfftw3f
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${CERES_LIBRARIES}
  yaml-cpp
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

