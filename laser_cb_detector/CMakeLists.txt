cmake_minimum_required(VERSION 2.8.3)
project(laser_cb_detector)

find_package(Boost REQUIRED signals thread)
find_package(catkin REQUIRED actionlib_msgs cv_bridge image_cb_detector settlerlib std_msgs)
catkin_package(DEPENDS actionlib_msgs cv_bridge image_cb_detector settlerlib std_msgs
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

add_action_files(DIRECTORY action FILES Config.action)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)
include_directories(include)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/laser_cb_detector/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
) 

# deal witht the library
add_library(${PROJECT_NAME} src/cv_laser_bridge.cpp
                            src/laser_cb_detector.cpp
                            src/laser_interval_calc.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# deal with some executables
add_executable(laser_cb_detector_node src/laser_cb_detector_node.cpp)
target_link_libraries(laser_cb_detector_node ${Boost_LIBRARIES}
                                             ${PROJECT_NAME}
                                             ${OpenCV_LIBRARIES}
)

add_executable(laser_interval_calc_node src/laser_interval_calc_node.cpp)
target_link_libraries(laser_interval_calc_node ${Boost_LIBRARIES}
                                               ${PROJECT_NAME}
                                               ${OpenCV_LIBRARIES}
)

install(TARGETS laser_cb_detector_node laser_interval_calc_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(test EXCLUDE_FROM_ALL)
