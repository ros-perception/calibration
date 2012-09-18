/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <calibration_msgs/IntervalStamped.h>
#include <laser_cb_detector/laser_interval_calc.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace laser_cb_detector;

void syncCallback(ros::Publisher* pub,
                  const calibration_msgs::DenseLaserSnapshotConstPtr& snapshot,
                  const calibration_msgs::CalibrationPatternConstPtr& features)
{
  calibration_msgs::IntervalStamped out;

  bool success;

  success = LaserIntervalCalc::computeInterval(*snapshot, *features, out.interval);

  if (!success)
  {
    ROS_WARN("Failed trying to compute interval. Not going to publish");
    return;
  }

  out.header.stamp = snapshot->header.stamp;
  pub->publish(out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_interval_calc");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<calibration_msgs::IntervalStamped>("laser_interval", 1);

  message_filters::Subscriber<calibration_msgs::DenseLaserSnapshot> snapshot_sub(nh, "snapshot", 1);
  message_filters::Subscriber<calibration_msgs::CalibrationPattern> features_sub(nh, "features", 1);
  message_filters::TimeSynchronizer<calibration_msgs::DenseLaserSnapshot,
                                    calibration_msgs::CalibrationPattern> sync(snapshot_sub, features_sub, 2);
  sync.registerCallback(boost::bind(&syncCallback, &pub, _1, _2));

  ros::spin();

  return 0;
}
