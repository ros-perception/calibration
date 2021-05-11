cmake_minimum_required(VERSION 2.8.3)
project(calibration_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs sensor_msgs std_msgs)

add_message_files(DIRECTORY msg
                  FILES CalibrationPattern.msg
                        CameraMeasurement.msg
                        ChainMeasurement.msg
                        DenseLaserObjectFeatures.msg
                        DenseLaserPoint.msg
                        DenseLaserSnapshot.msg
                        Interval.msg
                        IntervalStamped.msg
                        IntervalStatus.msg
                        JointStateCalibrationPattern.msg
                        LaserMeasurement.msg
                        RobotMeasurement.msg
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs)
