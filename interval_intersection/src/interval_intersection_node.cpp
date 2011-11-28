/**********************************************************************
 *
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

#include <vector>

#include "ros/ros.h"
#include "ros/console.h"
#include "calibration_msgs/Interval.h"
#include "interval_intersection/interval_intersection.hpp"

using namespace std;
using namespace interval_intersection;

void myPublish(ros::Publisher* pub, calibration_msgs::Interval interval)
{
  pub->publish(interval);
}

// Main
int main(int argc, char **argv)
{
  ros::init(argc,argv,"interval_intersection");

  if (argc < 2) {
    ROS_INFO_STREAM_NAMED("interval_intersection","At least one interval topic must be given.\n");
    return 0;
  }

  vector<string> names;
  for (int i=1; i<argc; i++) {
    names.push_back(argv[i]);
  }

  ros::NodeHandle nh;

  // Publisher
  ros::Publisher publisher = nh.advertise<calibration_msgs::Interval>("interval",1);
  // Intersector
  IntervalIntersector intersector(boost::bind(&myPublish, &publisher, _1));
  // Subscribe
  vector<ros::Subscriber> subscribers;
  for (size_t i=0; i < names.size(); i++) {
    ROS_DEBUG("Listening On Topic [%s]", names[i].c_str());
    subscribers.push_back(nh.subscribe<calibration_msgs::Interval>(names[i], 200, intersector.getNewInputStream()));
  }

  ROS_INFO_STREAM_NAMED("interval_intersection","Advertised interval");

  ros::spin();

  return 0;
}


