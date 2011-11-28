Header header
string laser_id
sensor_msgs/JointState[] joint_points

# True -> The extra debugging fields are populated
bool verbose

# Extra, partially processed data. Only needed for debugging
calibration_msgs/DenseLaserSnapshot snapshot
sensor_msgs/Image laser_image
calibration_msgs/CalibrationPattern image_features
calibration_msgs/JointStateCalibrationPattern joint_features
