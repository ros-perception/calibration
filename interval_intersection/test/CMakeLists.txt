# ********** Tests **********
catkin_add_gtest(interval_intersection_unittest interval_intersection_unittest.cpp)
target_link_libraries(interval_intersection_unittest ${PROJECT_NAME})
