cmake_minimum_required(VERSION 2.8.3)
project(calibration_launch)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

install(FILES launch/annotated_viewer.launch
              launch/chain_settler.launch
              launch/kinect_pipeline.launch
              launch/monocam_pipeline.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/capture_data
)

install(PROGRAMS scripts/urdf_pub.py
                 scripts/capture_exec.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
