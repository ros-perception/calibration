cmake_minimum_required(VERSION 2.8.3)
project(calibration_setup_helper)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS scripts/calibration_setup_helper.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
