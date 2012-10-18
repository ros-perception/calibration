# ********** Tests **********

catkin_add_gtest(sorted_deque_unittest sorted_deque_unittest.cpp)

catkin_add_gtest(interval_calc_unittest interval_calc_unittest.cpp)
target_link_libraries(interval_calc_unittest settlerlib)

catkin_add_gtest(deflated_unittest deflated_unittest.cpp)
target_link_libraries(deflated_unittest settlerlib)
