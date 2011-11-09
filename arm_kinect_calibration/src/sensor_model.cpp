/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Sachin Chitta */

#include <arm_kinect_calibration/sensor_model.h>

namespace arm_kinect_calibration
{

double SensorModel::generateRandomNumber(const double &min, const double &max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

SensorModel::SensorModel(const std::string &sensor_optical_frame_id,
                         const unsigned int &fov_degrees,
                         const double &min_range,
                         const double &max_range)
{
  initialize(sensor_optical_frame_id,fov_degrees,min_range,max_range);
}

SensorModel::~SensorModel(void)
{

}

void SensorModel::initialize(const std::string &sensor_optical_frame_id,
                             const unsigned int &fov_degrees,
                             const double &min_range,
                             const double &max_range)
{
  srand ( time(NULL) ); // initialize random seed: 
  sensor_optical_frame_id_ = sensor_optical_frame_id;
  min_range_ = min_range;
  max_range_ = max_range;
  fov_ = angles::from_degrees((double)fov_degrees);
}

std::string SensorModel::getFrameId()
{
  return sensor_optical_frame_id_;
}

void SensorModel::getRandomTargetPose(geometry_msgs::PoseStamped &target_pose)
{
  // Sample in the field of view of the sensor
  double range  = generateRandomNumber(min_range_,max_range_);
  double angle  = generateRandomNumber(0,fov_);
  double axis_x = generateRandomNumber(-1.0,1.0);
  double axis_y_sq = sqrt(1-axis_x*axis_x);
  double axis_y(axis_y_sq);
  if(generateRandomNumber(-1.0,1.0) >= 0.0)
    axis_y = -axis_y_sq;
  double axis_z = 0.0;

  tf::Point point1(0.0,0.0,0.0);
  tf::Point point2(0.0,0.0,range);

  tf::Vector3 axis(axis_x,axis_y,axis_z);
  tf::Quaternion target_quaternion(axis,angle);
  tf::Quaternion identity_quaternion = target_quaternion.getIdentity(); 

  // Move along the random ray (within the field of view)
  tf::Pose pose(target_quaternion,point1);
  tf::Pose pose2(identity_quaternion,point2);

  // Apply a random rotation (about body-fixed Z axis)
  double yaw = generateRandomNumber(-M_PI,M_PI);
  tf::Quaternion yaw_quaternion = tf::createQuaternionFromRPY(0.0,0.0,yaw);
  tf::Pose pose3(yaw_quaternion,point1);

  // Turn it around (so Z axis is pointing back at the camera)
  tf::Quaternion pitch_quaternion = tf::createQuaternionFromRPY(M_PI,0.0,0.0);
  tf::Pose pose4(pitch_quaternion,point1);

  pose = pose*pose2*pose3*pose4;
  tf::poseTFToMsg(pose,target_pose.pose);
  target_pose.header.frame_id = sensor_optical_frame_id_;
  target_pose.header.stamp = ros::Time(0.0);
}

bool SensorModel::isPointInView(geometry_msgs::Point &target_point,
                                double &angle)
{
  double dx = target_point.x;
  double dy = target_point.y;
  double dz = target_point.z;

  btVector3 z_axis(0.0,0.0,1.0);
  btVector3 target_vector(dx,dy,dz);

  double distance = target_vector.length();
  angle = fabs(target_vector.angle(z_axis));

  if(distance < min_range_ || distance > max_range_)
    return false;
  if(angle < fov_)
    return true;
  else
    return false;

}

}
