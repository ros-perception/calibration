# ********** Tests **********
catkin_add_gtest(interval_intersection_unittest interval_intersection_unittest.cpp)
add_dependencies(interval_intersection_unittest ${catkin_EXPORTED_TARGETS})
target_link_libraries(interval_intersection_unittest ${PROJECT_NAME})
