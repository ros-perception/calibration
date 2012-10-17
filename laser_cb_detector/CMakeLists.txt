cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

rosbuild_add_library(${PROJECT_NAME} src/cv_laser_bridge.cpp
                                     src/laser_cb_detector.cpp
                                     src/laser_interval_calc.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

rosbuild_add_boost_directories()
rosbuild_add_executable(laser_cb_detector_node src/laser_cb_detector_node.cpp)
target_link_libraries(laser_cb_detector_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})

rosbuild_add_executable(laser_interval_calc_node src/laser_interval_calc_node.cpp)
target_link_libraries(laser_interval_calc_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})
rosbuild_link_boost(laser_interval_calc_node signals)

rosbuild_genmsg()
add_subdirectory(test EXCLUDE_FROM_ALL)
