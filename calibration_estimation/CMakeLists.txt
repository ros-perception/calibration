cmake_minimum_required(VERSION 2.8.3)
project(calibration_estimation)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_add_nosetests(test/camera_unittest.py)
catkin_add_nosetests(test/checkerboard_unittest.py)
catkin_add_nosetests(test/joint_chain_unittest.py)
catkin_add_nosetests(test/full_chain_unittest.py)
catkin_add_nosetests(test/single_transform_unittest.py)
catkin_add_nosetests(test/tilting_laser_unittest.py)

catkin_add_nosetests(test/chain_sensor_unittest.py)
#catkin_add_nosetests(test/camera_chain_sensor_unittest.py)
catkin_add_nosetests(test/tilting_laser_sensor_unittest.py)

install(PROGRAMS  src/calibration_estimation/multi_step_cov_estimator.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
