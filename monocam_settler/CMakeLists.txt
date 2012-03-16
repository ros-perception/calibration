cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

rosbuild_add_library(${PROJECT_NAME} src/monocam_settler.cpp)

rosbuild_add_boost_directories()
rosbuild_add_executable(monocam_settler_action src/monocam_settler_action.cpp)
target_link_libraries(monocam_settler_action ${PROJECT_NAME})
rosbuild_link_libraries(monocam_settler_action thread)

add_subdirectory(test EXCLUDE_FROM_ALL)
