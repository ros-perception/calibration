set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********
rosbuild_add_gtest(test/interval_intersection_unittest interval_intersection_unittest.cpp)
target_link_libraries(test/interval_intersection_unittest ${PROJECT_NAME})