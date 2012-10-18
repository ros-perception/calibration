cmake_minimum_required(VERSION 2.8.3)
project(joint_states_settler)

find_package(Boost REQUIRED thread)
find_package(catkin REQUIRED actionlib actionlib_msgs rosconsole roscpp roscpp_serialization settlerlib std_msgs)
catkin_package(DEPENDS actionlib actionlib_msgs rosconsole roscpp roscpp_serialization settlerlib std_msgs
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)


add_action_files(DIRECTORY action FILES Config.action)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

# create the library
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
)
include_directories(include)

add_library(${PROJECT_NAME} src/joint_states_deflater.cpp
                                     src/joint_states_settler.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# add some executables
add_executable(joint_states_settler_action src/joint_states_settler_action.cpp)
target_link_libraries(joint_states_settler_action ${Boost_LIBRARIES}
                                                  ${PROJECT_NAME}
)

add_executable(view_interval src/view_interval.cpp)
target_link_libraries(view_interval ${catkin_LIBRARIES})

install(TARGETS joint_states_settler_action view_interval
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(test EXCLUDE_FROM_ALL)
