# ********** Tests **********
catkin_add_gtest(monocam_settler_unittest monocam_settler_unittest.cpp)
target_link_libraries(monocam_settler_unittest monocam_settler)
