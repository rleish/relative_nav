cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#PROJECT(relative_MEKF)

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

#enable OpenMP for threading:
#add_definitions(-fopenmp)

#enable pthreads:
add_definitions(-lpthread)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(relative_MEKF src/ros_server.cpp include/rel_estimator/ros_server.h)
rosbuild_add_library(relative_MEKF src/vodata.cpp include/rel_estimator/vodata.h)
rosbuild_add_library(relative_MEKF src/estimator.cpp include/rel_estimator/estimator.h)
rosbuild_add_library(relative_MEKF src/constants.cpp include/rel_estimator/constants.h)
rosbuild_add_library(relative_MEKF src/navnode.cpp include/rel_estimator/navnode.h)
rosbuild_add_library(relative_MEKF src/navedge.cpp include/rel_estimator/navedge.h)

include_directories(include/rel_estimator/statepacket.h)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(relative_MEKF src/main.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#OpenMP Thread Building Blocks
#target_link_libraries(kinect_visual_odometry gomp) 

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
