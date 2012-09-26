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

#ifndef LASER_CB_DETECTOR_LASER_CB_DETECTOR_H_
#define LASER_CB_DETECTOR_LASER_CB_DETECTOR_H_

#include <opencv/cv.h>
#include <calibration_msgs/DenseLaserSnapshot.h>
#include <laser_cb_detector/ConfigAction.h>
#include <laser_cb_detector/cv_laser_bridge.h>
#include <image_cb_detector/image_cb_detector.h>
#include <calibration_msgs/CalibrationPattern.h>
#include <sensor_msgs/Image.h>

namespace laser_cb_detector
{

class LaserCbDetector
{
public:
  LaserCbDetector();

  bool configure(const ConfigGoal& config);

  bool detect(const calibration_msgs::DenseLaserSnapshot& snapshot,
              calibration_msgs::CalibrationPattern& result);

  bool getImage(const calibration_msgs::DenseLaserSnapshot& snapshot, sensor_msgs::Image& ros_image);

private:
  bool configured_;
  ConfigGoal config_;
  CvLaserBridge bridge_;
  image_cb_detector::ImageCbDetector detector_;
};

}

#endif
