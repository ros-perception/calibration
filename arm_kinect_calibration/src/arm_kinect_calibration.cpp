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

#include <arm_kinect_calibration/target_pose_generator.h>

namespace arm_kinect_calibration
{

TargetConfigurationGenerator::TargetConfigurationGenerator(std::string &robot_description,
                                                           std::string &arm_name,
                                                           std::string &sensor_name,
                                                           std::string &target_name): collision_models_interface_(robot_description)
{
  arm_name_ = arm_name;
  sensor_name_ = sensor_name;
  target_name_ = target_name;
}

bool TargetConfigurationGenerator::initialize()
{
  if(!intitializeTarget())
    return false;
  if(!initializeSensorModel())
    return false;
  return true;
}

bool TargetConfigurationGenerator::initializeTarget(const std::string &target_name,
                                                    const std::string &arm_name)
{
  target_.initialize(target_name,arm_name);
}

bool TargetConfigurationGenerator::initializeSensorModel(const std::string &sensor_name)
{
  std::string sensor_optical_frame_id;
  double min_range, max_range;
  unsigned int fov_degrees;
  if(!node_handle_.getParam(sensor_name+"/optical_frame_id",sensor_optical_frame_id))
    return false;
  if(!node_handle_.param(sensor_name+"/fov_degrees",fov_degrees))
    return false;
  if(!node_handle_.param(sensor_name+"/min_range",min_range))
    return false;
  if(node_handle_.param(sensor_name+"/max_range",max_range))
    return false;
  sensor_model_.initialize(sensor_optical_frame_id,fov_degrees,min_range,max_range);
  return true;
}

bool TargetConfigurationGenerator::generateTargetConfiguration(geometry_msgs::PoseStamped &target_pose,
                                                               motion_planning_msgs::RobotState &robot_state)
{
  geometry_msgs::PoseStamped random_target_pose;
  sensor_model_.getRandomTargetPose(random_target_pose);

  planning_models::KinematicState *kinematic_state = collision_models_interface_.getPlanningSceneState();
  collision_models_interface_.convertPoseGivenWorldTransform(*kinematic_state,robot_frame_id_,random_target_pose.header,random_target_pose.pose,pose_stamped);

  target_.getEndEffectorPose(pose_stamped);
  
  if(!kinematics_solver_.getIK(pose_stamped,arm_state))
    return false;
  collision_models_interface_.updateState(arm_state);
  if(visibility_checker_.areVisible(target_.getFeaturePoints(pose_stamped)))
  {
    target_pose = pose_stamped;
    planning_environment::convertKinematicStateToRobotState(kinematic_state,ros::Time::now(), robot_frame_id_,robot_state);
    return true;
  }  
  return false;
}



}
