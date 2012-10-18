# ********** Tests **********
catkin_add_gtest(joint_states_deflater_unittest joint_states_deflater_unittest.cpp)
target_link_libraries(joint_states_deflater_unittest joint_states_settler)

catkin_add_gtest(joint_states_settler_unittest joint_states_settler_unittest.cpp)
target_link_libraries(joint_states_settler_unittest joint_states_settler)
