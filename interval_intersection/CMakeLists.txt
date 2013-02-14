cmake_minimum_required(VERSION 2.8.3)
project(interval_intersection)

find_package(Boost REQUIRED thread)
find_package(catkin REQUIRED actionlib actionlib_msgs calibration_msgs geometry_msgs rosconsole roscpp roscpp_serialization rostime std_msgs)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)
 
# generate the messages
add_action_files(DIRECTORY action FILES Config.action)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs)

# define the package
catkin_package(DEPENDS actionlib actionlib_msgs Boost calibration_msgs geometry_msgs rosconsole roscpp roscpp_serialization rostime std_msgs
               INCLUDE_DIRS include
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/interval_intersection/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/interval_intersection.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} calibration_msgs_gencpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# deal with the executable
add_executable(interval_intersection_node src/interval_intersection_node.cpp)
target_link_libraries(interval_intersection_node ${PROJECT_NAME})
add_dependencies(interval_intersection_node calibration_msgs_gencpp)

include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})
add_executable(interval_intersection_action src/interval_intersection_action.cpp)
target_link_libraries(interval_intersection_action ${Boost_LIBRARIES}
                                                   ${catkin_LIBRARIES}
                                                   ${PROJECT_NAME}
)
add_dependencies(interval_intersection_action ${PROJECT_NAME}_gencpp)

install(TARGETS interval_intersection_node interval_intersection_action
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(test EXCLUDE_FROM_ALL)
