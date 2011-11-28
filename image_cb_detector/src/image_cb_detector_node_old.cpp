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

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <image_cb_detector/image_cb_detector_old.h>
#include <sstream>

using namespace image_cb_detector;
using namespace std;

#define ROS_INFO_CONFIG(name) \
{\
  ostringstream ss;\
  ss << "[" << #name << "] -> " << config.name;\
  ROS_INFO(ss.str().c_str());\
}



image_cb_detector::ConfigGoal getParamConfig(ros::NodeHandle &n)
{
  image_cb_detector::ConfigGoal config;

  int num_x;
  n.param("~num_x", num_x, 3);
  config.num_x = (unsigned int) num_x;

  int num_y;
  n.param("~num_y", num_y, 3);
  config.num_y = (unsigned int) num_y;

  double spacing_x;
  n.param("~spacing_x", spacing_x, 1.0);
  config.spacing_x = spacing_x;

  double spacing_y;
  n.param("~spacing_y", spacing_y, 1.0);
  config.spacing_y = spacing_y;

  double width_scaling;
  n.param("~width_scaling",  width_scaling,  1.0);
  config.width_scaling = width_scaling;

  double height_scaling;
  n.param("~height_scaling", height_scaling, 1.0);
  config.height_scaling = height_scaling;

  int subpixel_window;
  n.param("~subpixel_window", subpixel_window, 2);
  config.subpixel_window = subpixel_window;

  n.param("~subpixel_zero_zone", config.subpixel_zero_zone, -1);

  ROS_INFO_CONFIG(num_x);
  ROS_INFO_CONFIG(num_y);
  ROS_INFO_CONFIG(spacing_x);
  ROS_INFO_CONFIG(spacing_y);
  ROS_INFO_CONFIG(width_scaling);
  ROS_INFO_CONFIG(height_scaling);
  ROS_INFO_CONFIG(subpixel_window);
  ROS_INFO_CONFIG(subpixel_zero_zone);

  return config;
}

void imageCallback(ros::Publisher* pub, ImageCbDetectorOld* detector, const sensor_msgs::ImageConstPtr& image)
{
  calibration_msgs::CalibrationPattern features;
  bool success;
  success = detector->detect(image, features);

  if (!success)
  {
    ROS_ERROR("Error trying to detect checkerboard, not going to publish CalibrationPattern");
    return;
  }
  pub->publish(features);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh;

  ImageCbDetectorOld detector;

  detector.configure(getParamConfig(nh));

  image_transport::ImageTransport it_(nh);
  ros::Publisher pub = nh.advertise<calibration_msgs::CalibrationPattern>("features",1);
  image_transport::Subscriber sub = it_.subscribe("image", 2, boost::bind(&imageCallback, &pub, &detector, _1));

  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();
  //ros::spin();
  return 0;
}
