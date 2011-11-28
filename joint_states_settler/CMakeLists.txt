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
# rosbuild_gensrv()

rosbuild_add_library(${PROJECT_NAME} src/joint_states_deflater.cpp
                                     src/joint_states_settler.cpp)


rosbuild_add_executable(joint_states_settler_node src/joint_states_settler_node.cpp)
target_link_libraries(joint_states_settler_node ${PROJECT_NAME})

rosbuild_add_executable(joint_states_settler_action src/joint_states_settler_action.cpp)
target_link_libraries(joint_states_settler_action ${PROJECT_NAME})

rosbuild_add_executable(view_interval src/view_interval.cpp)

# target_link_libraries(${PROJECT_NAME} another_library)
# rosbuild_add_boost_directories()
# rosbuild_link_boost(${PROJECT_NAME} thread)
# rosbuild_add_executable(example examples/example.cpp)
# target_link_libraries(example ${PROJECT_NAME})

add_subdirectory(test EXCLUDE_FROM_ALL)