# ********** Tests **********

catkin_add_gtest(cv_laser_bridge_unittest     cv_laser_bridge_unittest.cpp)
target_link_libraries(cv_laser_bridge_unittest laser_cb_detector)
add_dependencies(cv_laser_bridge_unittest calibration_msgs_gencpp)

catkin_add_gtest(laser_cb_detector_unittest     laser_cb_detector_unittest.cpp)
target_link_libraries(laser_cb_detector_unittest laser_cb_detector)

catkin_add_gtest(laser_interval_calc_unittest     laser_interval_calc_unittest.cpp)
target_link_libraries(laser_interval_calc_unittest laser_cb_detector)
