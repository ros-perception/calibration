/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

//! \author Vijay Pradeep

#include <cstdio>
#include <ros/ros.h>
#include <joint_states_settler/joint_states_settler.h>

using namespace joint_states_settler;

/**
 * Get the configuration of the settler off the param server.
 * We do this by incrementally generating param names until we
 * can't find a param on the param server
 */
joint_states_settler::ConfigGoal getParamConfig(ros::NodeHandle& n)
{
  joint_states_settler::ConfigGoal config;
  config.joint_names.clear() ;
  bool found_joint = true ;
  int joint_count = 0 ;

  char param_buf[1024] ;
  while(found_joint)
  {
    sprintf(param_buf, "~joint_name_%02u", joint_count) ;
    std::string param_name = param_buf ;
    std::string cur_joint_name ;
    found_joint = n.getParam(param_name, cur_joint_name) ;
    if (found_joint)
    {
      ROS_INFO("[%s] -> %s", param_name.c_str(), cur_joint_name.c_str()) ;
      config.joint_names.push_back(cur_joint_name) ;

      // Get the joint tolerance only if we found the joint name
      sprintf(param_buf, "~joint_tol_%02u", joint_count) ;
      double cur_tol;
      bool found_tol;
      found_tol = n.getParam(std::string(param_buf), cur_tol) ;
      if (!found_tol)
        ROS_FATAL("Could not find parameter [%s]", param_buf);
      ROS_INFO("[%s]  -> %.3f", param_buf, cur_tol) ;
      config.tolerances.push_back(cur_tol);
    }
    else
      ROS_DEBUG("Did not find param [%s]", param_name.c_str()) ;

    joint_count++ ;
  }

  double max_step;
  n.param("~max_step", max_step, 1000.0);
  config.max_step = ros::Duration(max_step);
  ROS_INFO("max step: [%.3fs]", config.max_step.toSec());

  int cache_size;
  n.param("~cache_size", cache_size, 1000);
  if (cache_size < 0)
    ROS_FATAL("cache_size < 0. (cache_size==%i)", cache_size);
  config.cache_size = (unsigned int) cache_size;
  ROS_INFO("cache_size: [%u]", config.cache_size);

  return config;
}

void jointStatesCallback(ros::Publisher* pub, JointStatesSettler* settler, const sensor_msgs::JointStateConstPtr& msg)
{
  pub->publish(settler->add(msg));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_settler");

  ros::NodeHandle n;

  // Set up the JointStatesSettler
  joint_states_settler::ConfigGoal config = getParamConfig(n);
  JointStatesSettler settler;
  settler.configure(config);

  // Output
  ros::Publisher pub = n.advertise<calibration_msgs::Interval>("settled", 1);

  // Input

  boost::function<void (const sensor_msgs::JointStateConstPtr&)> cb = boost::bind(&jointStatesCallback, &pub, &settler, _1);
  ros::Subscriber sub = n.subscribe(std::string("joint_states"), 1, cb);

  ros::spin();
}
