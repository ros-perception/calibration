cmake_minimum_required(VERSION 2.8.3)
project(monocam_settler)

find_package(Boost REQUIRED thread)
find_package(catkin REQUIRED actionlib actionlib_msgs rosconsole roscpp_serialization settlerlib std_msgs)
catkin_package(DEPENDS actionlib actionlib_msgs rosconsole roscpp_serialization settlerlib std_msgs
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)



add_action_files(DIRECTORY action FILES Config.action)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

# deal with the include
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/monocam_settler/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# create the libraries
add_library(${PROJECT_NAME} src/monocam_settler.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


# deal with the executable
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
add_executable(monocam_settler_action src/monocam_settler_action.cpp)
target_link_libraries(monocam_settler_action ${Boost_LIBRARIES}
                                             ${catkin_LIBRARIES}
                                             ${PROJECT_NAME}
)
install(TARGETS monocam_settler_action
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(test EXCLUDE_FROM_ALL)
