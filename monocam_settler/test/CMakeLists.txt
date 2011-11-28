set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********
rosbuild_add_gtest(test/monocam_settler_unittest monocam_settler_unittest.cpp)
target_link_libraries(test/monocam_settler_unittest monocam_settler)
