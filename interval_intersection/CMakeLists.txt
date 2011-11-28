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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/interval_intersection.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(interval_intersection src/interval_intersection.cpp src/interval_intersector.cpp)

rosbuild_add_executable(interval_intersection_node src/interval_intersection_node.cpp)
target_link_libraries(interval_intersection_node ${PROJECT_NAME})

rosbuild_add_executable(interval_intersection_action src/interval_intersection_action.cpp)
target_link_libraries(interval_intersection_action ${PROJECT_NAME})

#rosbuild_add_executable(test_interval_intersector test/test_interval_intersector.cpp src/interval_intersector.cpp)
#rosbuild_add_gtest(test_interval_intersector test/test_interval_intersector.cpp src/interval_intersector.cpp)
#set_target_properties(test_interval_intersector PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

add_subdirectory(test EXCLUDE_FROM_ALL)
