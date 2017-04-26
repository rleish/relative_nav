cmake_minimum_required(VERSION 2.4.6)

#PROJECT(kinect_visual_odometry)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

#include Eigen:
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#get opencv
find_package(OpenCV REQUIRED)

#enable OpenMP for threading:
add_definitions(-fopenmp)

# Qt ########################
# http://qtnode.net/wiki/Qt4_with_cmake
find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML FALSE)
SET(QT_USE_QTOPENGL FALSE)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_library(kinect_visual_odometry src/ros_relay.cpp include/ros_relay.h)
rosbuild_add_library(kinect_visual_odometry src/pose_estimator.cpp include/pose_estimator.h)
rosbuild_add_library(kinect_visual_odometry src/image_display.cpp include/image_display.h)
rosbuild_add_library(kinect_visual_odometry src/ransac.cpp include/ransac.h)
#rosbuild_add_library(kinect_visual_odometry src/lsh.cpp include/lsh.h)

#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(kinect_visual_odometry boost_signals)

rosbuild_add_executable(kinect_visual_odometry src/main.cpp)

#target_link_libraries(example ${PROJECT_NAME})
target_link_libraries(kinect_visual_odometry ${QT_LIBRARIES})

#set g2o libs
#SET(G2O_LIBS cholmod g2o_core g2o_stuff g2o_types_slam3d g2o_types_sba g2o_solver_cholmod g2o_solver_pcg g2o_solver_csparse g2o_incremental)
#target_link_libraries(kinect_visual_odometry ${G2O_LIBS})

target_link_libraries(kinect_visual_odometry gomp) # OpenMP Thread Building Blocks

target_link_libraries(kinect_visual_odometry ${OpenCV_LIBS})

rosbuild_link_boost(kinect_visual_odometry signals)


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)




