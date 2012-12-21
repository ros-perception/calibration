/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008-2012, Willow Garage, Inc.
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

#include <laser_cb_detector/laser_cb_detector.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
//#include <highgui.h>

using namespace std;
using namespace laser_cb_detector;

LaserCbDetector::LaserCbDetector() : configured_(false) {}

bool LaserCbDetector::configure(const ConfigGoal& config)
{
  config_ = config;
  image_cb_detector::ConfigGoal image_cfg;
  // TODO: setup message

  image_cfg.num_x = config.num_x;
  image_cfg.num_y = config.num_y;
  image_cfg.spacing_x = config.spacing_x;
  image_cfg.spacing_y = config.spacing_y;

  image_cfg.width_scaling = config.width_scaling;
  image_cfg.height_scaling = config.height_scaling;

  image_cfg.subpixel_window = config.subpixel_window;
  image_cfg.subpixel_zero_zone = config.subpixel_zero_zone;

  detector_.configure(image_cfg);
  return true;
}

bool LaserCbDetector::detect(const calibration_msgs::DenseLaserSnapshot& snapshot,
                             calibration_msgs::CalibrationPattern& result)
{
  // ***** Convert the snapshot into an image, based on intensity window in config *****
  if(!bridge_.fromIntensity(snapshot, config_.min_intensity, config_.max_intensity))
    return false;
  IplImage* image = bridge_.toIpl();

  if (config_.flip_horizontal)
  {
    ROS_DEBUG("Flipping image");
    cvFlip(image, NULL, 1);
  }
  else
    ROS_DEBUG("Not flipping image");

  cv_bridge::CvImage cv_image(snapshot.header, "mono8", image);
  sensor_msgs::ImagePtr ros_image = cv_image.toImageMsg();
  if(detector_.detect(ros_image, result)){
    if (config_.flip_horizontal){
      for(int i=0; i < result.image_points.size(); i++)
        result.image_points[i].x = image->width - result.image_points[i].x - 1;
    }
    return true;
  }else
    return false;
}

bool LaserCbDetector::getImage(const calibration_msgs::DenseLaserSnapshot& snapshot, sensor_msgs::Image& ros_image)
{
  if(!bridge_.fromIntensity(snapshot, config_.min_intensity, config_.max_intensity))
  {
    ROS_ERROR("Error building IplImage from DenseLaserSnapshot's intensity data");
    return false;
  }
  IplImage* image = bridge_.toIpl();

  cv_bridge::CvImage(snapshot.header, "mono8", image).toImageMsg(ros_image);

  return true;
}

