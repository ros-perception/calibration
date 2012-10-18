# ********** Tests **********

catkin_add_gtest(sorted_deque_unittest sorted_deque_unittest.cpp)
target_link_libraries(sorted_deque_unittest ${catkin_LIBRARIES}
                                            ${PROJECT_NAME}
)

catkin_add_gtest(interval_calc_unittest interval_calc_unittest.cpp)
target_link_libraries(interval_calc_unittest ${catkin_LIBRARIES}
                                             ${PROJECT_NAME}
)

catkin_add_gtest(deflated_unittest deflated_unittest.cpp)
target_link_libraries(deflated_unittest ${catkin_LIBRARIES}
                                        ${PROJECT_NAME}
)
