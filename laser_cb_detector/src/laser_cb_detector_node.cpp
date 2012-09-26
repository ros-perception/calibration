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

#include <ros/console.h>
#include <ros/ros.h>
#include <laser_cb_detector/laser_cb_detector.h>
#include <sstream>
#include <actionlib/server/simple_action_server.h>

using namespace laser_cb_detector;
using namespace std;


#define ROS_INFO_CONFIG(name) \
{\
  ostringstream ss;\
  ss << "[" << #name << "] -> " << config.name;\
  ROS_INFO("%s", ss.str().c_str());\
}

laser_cb_detector::ConfigGoal getParamConfig(ros::NodeHandle &n)
{
  laser_cb_detector::ConfigGoal config;

  ros::NodeHandle pn("~");

  int num_x;
  pn.param("num_x", num_x, 3);
  config.num_x = (unsigned int) num_x;

  int num_y;
  pn.param("num_y", num_y, 3);
  config.num_y = (unsigned int) num_y;

  double spacing_x;
  pn.param("spacing_x", spacing_x, 1.0);
  config.spacing_x = spacing_x;

  double spacing_y;
  pn.param("spacing_y", spacing_y, 1.0);
  config.spacing_y = spacing_y;

  double width_scaling;
  pn.param("width_scaling",  width_scaling,  1.0);
  config.width_scaling = width_scaling;

  double height_scaling;
  pn.param("height_scaling", height_scaling, 1.0);
  config.height_scaling = height_scaling;

  double min_intensity;
  pn.param("min_intensity", min_intensity, 500.0);
  config.min_intensity = min_intensity;

  double max_intensity;
  pn.param("max_intensity", max_intensity, 5000.0);
  config.max_intensity = max_intensity;

  int subpixel_window;
  pn.param("subpixel_window", subpixel_window, 2);
  config.subpixel_window = subpixel_window;

  pn.param("subpixel_zero_zone", config.subpixel_zero_zone, -1);

  int flip_horizontal;
  pn.param("flip_horizontal", flip_horizontal, 0);
  if (flip_horizontal)
    config.flip_horizontal = 1;
  else
    config.flip_horizontal = 0;

  ROS_INFO_CONFIG(num_x);
  ROS_INFO_CONFIG(num_y);
  ROS_INFO_CONFIG(spacing_x);
  ROS_INFO_CONFIG(spacing_y);
  ROS_INFO_CONFIG(width_scaling);
  ROS_INFO_CONFIG(height_scaling);
  ROS_INFO_CONFIG(min_intensity);
  ROS_INFO_CONFIG(max_intensity);
  ROS_INFO_CONFIG(subpixel_window);
  ROS_INFO_CONFIG(subpixel_zero_zone);
  //ROS_INFO_CONFIG(reflect_model);
  ROS_INFO("[flip_horizontal]->%u", config.flip_horizontal);

  return config;
}

class LaserCbDetectorAction {
   public:
      LaserCbDetectorAction() : as_("cb_detector_config", false) {
         // Set up the LaserCbDetector
         laser_cb_detector::ConfigGoal config = getParamConfig(nh_);
         detector_.configure(config);

         // Set up the action server for config
         as_.registerGoalCallback( boost::bind(&LaserCbDetectorAction::goalCallback, this) );
         as_.registerPreemptCallback( boost::bind(&LaserCbDetectorAction::preemptCallback, this) );

         // set up feature publisher
         pub_ = nh_.advertise<calibration_msgs::CalibrationPattern>("laser_checkerboard", 1);
         // set up image publisher
         image_pub_ = nh_.advertise<sensor_msgs::Image>("image", 1);

         // subscribe to laser snapshotter
         sub_ = nh_.subscribe("snapshot", 1, &LaserCbDetectorAction::snapshotCallback, this );

         // start the action server
         as_.start();
      }

      void goalCallback() {
        boost::mutex::scoped_lock lock(run_mutex_);

        if(as_.isActive()) {
           as_.setPreempted();
        }

        laser_cb_detector::ConfigGoalConstPtr goal = as_.acceptNewGoal();

        bool success = detector_.configure(*goal);

        if( !success ) {
           as_.setAborted();
        }
      }

      void preemptCallback() {
        boost::mutex::scoped_lock lock(run_mutex_);
        // nothing special to do on preempt
        as_.setPreempted();
      }

      void snapshotCallback(const calibration_msgs::DenseLaserSnapshotConstPtr& msg)
      {
        boost::mutex::scoped_lock lock(run_mutex_);

        bool detect_result;
        calibration_msgs::CalibrationPattern result;
        detect_result = detector_.detect(*msg, result);
      
        if (!detect_result)
          ROS_ERROR("Error during checkerboard detection. (This error is worse than simply not seeing a checkerboard");
        else
        {
          result.header.stamp = msg->header.stamp;
          pub_.publish(result);
        }
      
        sensor_msgs::Image image;
        if(detector_.getImage(*msg, image))
        {
          image.header.stamp = msg->header.stamp;
          image_pub_.publish(image);
        }
        else
          ROS_ERROR("Error trying to generate ROS image");
      }

   private:
      boost::mutex run_mutex_;
      ros::NodeHandle nh_;
      ros::Publisher pub_;
      ros::Publisher image_pub_;
      ros::Subscriber sub_;
      LaserCbDetector detector_;
      actionlib::SimpleActionServer<laser_cb_detector::ConfigAction> as_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_cb_detector");
  LaserCbDetectorAction detector_action;
  ros::spin();
  return 0;
}
