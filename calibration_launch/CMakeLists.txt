cmake_minimum_required(VERSION 2.8.3)
project(calibration_launch)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

install(FILES capture_data/annotated_viewer.launch
              capture_data/chain_settler.launch
              capture_data/kinect_pipeline.launch
              capture_data/monocam_pipeline.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/capture_data
)

install(PROGRAMS  capture_data/urdf_pub.py
                  src/capture_executive/capture_exec.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
