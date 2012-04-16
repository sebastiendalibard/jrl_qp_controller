cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
INCLUDE(cmake/base.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
if ( COMMAND cmake_policy )
  cmake_policy( SET CMP0003 NEW )  
endif()
find_package(CGAL QUIET COMPONENTS Core )
if ( CGAL_FOUND )
  include( ${CGAL_USE_FILE} )
  include( CGAL_CreateSingleSourceCGALProgram )
else()
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
endif()

ADD_REQUIRED_DEPENDENCY("hpp-gik >= 2.7")

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries

rosbuild_add_library(${PROJECT_NAME} 
  src/problem.cc 
  src/min-torque-task.cc 
  src/feature-task.cc
  src/gik-feature-task.cc
  src/contact-constraint.cc 
  src/position-task.cc
  src/transformation-task.cc
  src/com-task.cc
  src/controller.cc)

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-gik)

#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(simple-controller examples/simple-controller.cc)
rosbuild_add_executable(humanoid-controller examples/humanoid-posture-controller.cc)
target_link_libraries(simple-controller ${PROJECT_NAME})
target_link_libraries(humanoid-controller ${PROJECT_NAME})