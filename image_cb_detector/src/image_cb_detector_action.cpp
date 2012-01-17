/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009-2012, Willow Garage, Inc.
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

#include <boost/thread.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_cb_detector/image_cb_detector.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <image_cb_detector/ConfigAction.h>
#include <calibration_msgs/Interval.h>

using namespace image_cb_detector;

class ImageCbDetectorAction
{
public:
  ImageCbDetectorAction() : as_("cb_detector_config", false), it_(nh_)
  {
    as_.registerGoalCallback( boost::bind(&ImageCbDetectorAction::goalCallback, this) );
    as_.registerPreemptCallback( boost::bind(&ImageCbDetectorAction::preemptCallback, this) );

    pub_ = nh_.advertise<calibration_msgs::CalibrationPattern>("features",1);
    sub_ = it_.subscribe("image", 2, boost::bind(&ImageCbDetectorAction::imageCallback, this, _1));
    as_.start();
  }

  void goalCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    // Stop the previously running goal (if it exists)
    if (as_.isActive())
      as_.setPreempted();

    // Get the new goal from the action server
    image_cb_detector::ConfigGoalConstPtr goal = as_.acceptNewGoal();
    assert(goal);

    // Try to reconfigure the settler object
    bool success = detector_.configure(*goal);

    // Detect possible failure
    if (!success)
      as_.setAborted();
  }

  void preemptCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    // Don't need to do any cleanup. Immeadiately turn it off
    as_.setPreempted();
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& image)
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    if (as_.isActive())
    {
      calibration_msgs::CalibrationPattern features;
      bool success;
      success = detector_.detect(image, features);

      if (!success)
      {
        ROS_ERROR("Error trying to detect checkerboard, not going to publish CalibrationPattern");
        return;
      }

      pub_.publish(features);
    }
  }

private:
  boost::mutex run_mutex_;
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<image_cb_detector::ConfigAction> as_;
  ImageCbDetector detector_;

  ros::Publisher pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_cb_detector_action");
  ros::NodeHandle n;
  ImageCbDetectorAction detector_action;
  ros::spin();
  return 0;
}
