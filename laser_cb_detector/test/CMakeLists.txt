set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********

rosbuild_add_gtest(test/cv_laser_bridge_unittest     cv_laser_bridge_unittest.cpp)
target_link_libraries(test/cv_laser_bridge_unittest laser_cb_detector)


rosbuild_add_gtest(test/laser_cb_detector_unittest     laser_cb_detector_unittest.cpp)
target_link_libraries(test/laser_cb_detector_unittest laser_cb_detector)

rosbuild_add_gtest(test/laser_interval_calc_unittest     laser_interval_calc_unittest.cpp)
target_link_libraries(test/laser_interval_calc_unittest laser_cb_detector)
