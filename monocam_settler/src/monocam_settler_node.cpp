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

#include <ros/ros.h>
#include <monocam_settler/monocam_settler.h>

using namespace monocam_settler;

monocam_settler::ConfigGoal getParamConfig(ros::NodeHandle& n)
{
  monocam_settler::ConfigGoal config;

  n.param("~tolerance", config.tolerance, 1.0);
  ROS_INFO("tolerance: %.3f", config.tolerance);

  bool ignore_failures;
  n.param("~ignore_failures", ignore_failures, true);
  config.ignore_failures = ignore_failures;
  if (config.ignore_failures)
    ROS_INFO("Ignore Failures: True");
  else
    ROS_INFO("Ignore Failures: False");

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

void msgCallback(ros::Publisher* pub, MonocamSettler* settler, const calibration_msgs::CalibrationPatternConstPtr& msg)
{
  bool success;

  calibration_msgs::Interval interval;
  success = settler->add(msg, interval);

  if (success)
    pub->publish(interval);
  else
  {
    interval.start = msg->header.stamp;
    interval.end =   msg->header.stamp;
    pub->publish(interval);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "monocam_settler");

  ros::NodeHandle n;

  // Set up the MonocamSettler
  monocam_settler::ConfigGoal config = getParamConfig(n);
  MonocamSettler settler;
  settler.configure(config);

  // Output
  ros::Publisher pub = n.advertise<calibration_msgs::Interval>("settled", 1);

  // Input
  boost::function<void (const calibration_msgs::CalibrationPatternConstPtr&)> cb = boost::bind(&msgCallback, &pub, &settler, _1);
  ros::Subscriber sub = n.subscribe(std::string("features"), 1, cb);

  ros::spin();
}
