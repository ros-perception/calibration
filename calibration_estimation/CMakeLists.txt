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

rosbuild_add_pyunit(test/camera_unittest.py)
rosbuild_add_pyunit(test/checkerboard_unittest.py)
rosbuild_add_pyunit(test/joint_chain_unittest.py)
rosbuild_add_pyunit(test/full_chain_unittest.py)
rosbuild_add_pyunit(test/single_transform_unittest.py)
#rosbuild_add_pyunit(test/tilting_laser_unittest.py)

#rosbuild_add_pyunit(test/chain_sensor_unittest.py)
#rosbuild_add_pyunit(test/camera_chain_sensor_unittest.py)
#rosbuild_add_pyunit(test/tilting_laser_sensor_unittest.py)
