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

//! \author Vijay Pradeep

#include <laser_cb_detector/laser_interval_calc.h>
#include <ros/console.h>
#include <vector>
#include <algorithm>

using namespace laser_cb_detector;
using namespace std;

bool LaserIntervalCalc::computeInterval(const calibration_msgs::DenseLaserSnapshot& snapshot,
                                        const calibration_msgs::CalibrationPattern& features,
                                        calibration_msgs::Interval& result)
{
  const unsigned int N = features.image_points.size();

  vector<ros::Time> min_times, max_times;
  min_times.resize(N);
  max_times.resize(N);

  if (N == 0)
  {
    result.start = snapshot.header.stamp;
    result.end   = snapshot.header.stamp;
    return true;
  }

  // Find the min and max time bounds for each image point. Store this in vectors min_times and max_times
  for (unsigned int i=0; i<N; i++)
  {
    // Figure out which scan happened first
    int x_rounded = (int) features.image_points[i].x;
    int y_rounded = (int) features.image_points[i].y;

    // Don't need an x axis range check, since we don't dereference anything based on this index

    // Error checking on Y axis
    if (features.image_points[i].y <= 0.0 || y_rounded >= (int) snapshot.num_scans-1)
    {
      ROS_ERROR("Image point #%u (%.2f, %.2f) is outside of Y range (0.00, %u)", i,
                features.image_points[i].x, features.image_points[i].y, snapshot.num_scans-1);
      return false;
    }

    ros::Time min_scan_start = min( snapshot.scan_start[y_rounded],
                                    snapshot.scan_start[y_rounded+1] );
    ros::Time max_scan_start = max( snapshot.scan_start[y_rounded],
                                    snapshot.scan_start[y_rounded+1] );

    min_times[i] = min_scan_start + ros::Duration(snapshot.time_increment * x_rounded);
    max_times[i] = max_scan_start + ros::Duration(snapshot.time_increment * (x_rounded+1));
  }

  // Compute the min and max times over both vectors
  ros::Time min_time = min_times[0];
  ros::Time max_time = max_times[0];

  for (unsigned int i=0; i<N; i++)
  {
    min_time = min (min_time, min_times[i]);
    max_time = max (max_time, max_times[i]);
  }

  result.start = min_time;
  result.end   = max_time;

  return true;
}
