cmake_minimum_required(VERSION 2.8.3)
project(settlerlib)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED calibration_msgs rosconsole rostime)
catkin_package(DEPENDS Boost calibration_msgs rosconsole rostime
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

# common commands for building c++ executables and libraries
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/settlerlib/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
 
add_library(${PROJECT_NAME} src/interval_calc.cpp
                            src/deflated.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} calibration_msgs_gencpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_subdirectory(test EXCLUDE_FROM_ALL)
