/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

//! \author Michael Ferguson

#include <boost/thread.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_cb_detector/image_cb_detector.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_cb_detector/ConfigAction.h>
#include <calibration_msgs/Interval.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> CameraSyncPolicy;

class RgbdCbDetectorAction
{
public:
  RgbdCbDetectorAction(ros::NodeHandle & n) : nh_(n),
                           as_("cb_detector_config", false),
                           image_sub_ (nh_, "image", 3),
                           cloud_sub_(nh_, "points", 3),
                           sync_(CameraSyncPolicy(10), image_sub_, cloud_sub_)

  {
    as_.registerGoalCallback( boost::bind(&RgbdCbDetectorAction::goalCallback, this) );
    as_.registerPreemptCallback( boost::bind(&RgbdCbDetectorAction::preemptCallback, this) );

    pub_ = nh_.advertise<calibration_msgs::CalibrationPattern>("features",1);
    sync_.registerCallback(boost::bind(&RgbdCbDetectorAction::cameraCallback, this, _1, _2));

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

  void cameraCallback ( const sensor_msgs::ImageConstPtr& image,
                        const sensor_msgs::PointCloud2ConstPtr& depth)
//  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    if (as_.isActive())
    {
      calibration_msgs::CalibrationPattern features;
      bool success;
      // convert cloud to PCL
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      pcl::fromROSMsg(*depth, cloud);
      success = detector_.detect(image, features);
      if (!success)
      {
        ROS_ERROR("Error trying to detect checkerboard, not going to publish CalibrationPattern");
        return;
      }

      for(size_t i = 0; i< features.image_points.size(); i++){
        geometry_msgs::Point *p = &(features.image_points[i]);
        pcl::PointXYZRGB pt = cloud((int)(p->x+0.5), (int)(p->y+0.5));
        if( isnan(pt.x) || isnan(pt.y) || isnan(pt.z) ) {
          ROS_ERROR("Invalid point in checkerboard, not going to publish CalibrationPattern");
          return;
        }
        // z is set to distance
        p->z = sqrt( (pt.x*pt.x) + (pt.y*pt.y) + (pt.z*pt.z) );
      }

      pub_.publish(features);
    }
  }

private:
  boost::mutex run_mutex_;
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<image_cb_detector::ConfigAction> as_;
  image_cb_detector::ImageCbDetector detector_;

  message_filters::Subscriber<sensor_msgs::Image> image_sub_; 
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Synchronizer<CameraSyncPolicy> sync_;

  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_cb_detector_action");
  ros::NodeHandle n;
  RgbdCbDetectorAction detector_action(n);
  ros::spin();
  return 0;
}

