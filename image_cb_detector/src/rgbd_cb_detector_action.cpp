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

//! \author Michael Ferguson, David Gossow

#include <boost/thread.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_cb_detector/image_cb_detector.h>
#include <image_cb_detector/depth_to_pointcloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_cb_detector/ConfigAction.h>
#include <calibration_msgs/Interval.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/image_encodings.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> CameraSyncPolicy;

class RgbdCbDetectorAction
{
public:
  RgbdCbDetectorAction(ros::NodeHandle & n) : nh_(n),
                           as_("cb_detector_config", false),
                           image_sub_ (nh_, "image", 3),
                           depth_sub_(nh_, "depth", 3),
                           caminfo_sub_(nh_, "camera_info", 3),
                           sync_(CameraSyncPolicy(10), image_sub_, depth_sub_, caminfo_sub_)
  {
    as_.registerGoalCallback( boost::bind(&RgbdCbDetectorAction::goalCallback, this) );
    as_.registerPreemptCallback( boost::bind(&RgbdCbDetectorAction::preemptCallback, this) );

    pub_ = nh_.advertise<calibration_msgs::CalibrationPattern>("features",1);
    sync_.registerCallback(boost::bind(&RgbdCbDetectorAction::cameraCallback, this, _1, _2, _3));

    as_.start();

    last_sample_invalid_ = false;
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

  void cameraCallback ( const sensor_msgs::ImageConstPtr& image_msg,
                        const sensor_msgs::ImageConstPtr& depth_msg,
                        const sensor_msgs::CameraInfoConstPtr& caminfo_msg)
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    std::ostringstream s;
    s << image_sub_.getTopic() << ": ";

    if (as_.isActive())
    {
      // detect checkerboard corners
      calibration_msgs::CalibrationPattern features;
      bool success;
      success = detector_.detect(image_msg, features);
      if (!success)
      {
        ROS_ERROR_STREAM(s.str()<<"Error trying to detect checkerboard, not going to publish CalibrationPattern");
        last_sample_invalid_ = true;
        return;
      }
      if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
        ROS_ERROR_STREAM("Depth image must be 32-bit floating point (encoding '32FC1'), but has encoding '" << depth_msg->encoding << "'");
        last_sample_invalid_ = true;
        return;
      }

      // prepare data
      cloud_converter_.initialize( image_msg, caminfo_msg );
      const float* depth_ptr = reinterpret_cast<const float*>(&depth_msg->data[0]);
      std::size_t width = depth_msg->width;
      std::size_t height = depth_msg->height;

      // make a pointcloud from the checkerboard corners
      std::vector<cv::Point3f> corner_cloud;
      for(size_t i = 0; i< features.image_points.size(); i++){
        geometry_msgs::Point pixel = features.image_points[i];
        float depth = *(depth_ptr+width*(unsigned int)pixel.y+(unsigned int)pixel.x);
        if ( isnan(depth) )
        {
          continue;
        }

        cv::Point3f point;
        cloud_converter_.depthTo3DPoint( pixel, depth, point );
        corner_cloud.push_back( point );
      }

      if( corner_cloud.size() < features.image_points.size()/2 ) {
        ROS_ERROR_STREAM(s.str() << "More than 50% missing 3d points in checkerboard, not going to publish CalibrationPattern");
        last_sample_invalid_ = true;
        return;
      }

      // estimate plane
      cv::Vec3f n;
      float d;
      float best_ratio = 0;
      cv::RNG rng;
      for(size_t i = 0; i < 100; ++i) {
        // Get 3 random points
        for(int i=0;i<3;++i) {
          std::swap(corner_cloud[i], corner_cloud[rng.uniform(i+1, int(corner_cloud.size()))]);
        }
        // Compute the plane from those
        cv::Vec3f nrm = cv::Vec3f(corner_cloud[2]-corner_cloud[0]).cross(cv::Vec3f(corner_cloud[1])-cv::Vec3f(corner_cloud[0]));
        nrm = nrm / cv::norm(nrm);
        cv::Vec3f x0(corner_cloud[0]);

        float p_to_plane_thresh = 0.01;
        int num_inliers = 0;

        // Check the number of inliers
        for (size_t i=0; i<corner_cloud.size(); ++i) {
            cv::Vec3f w = cv::Vec3f(corner_cloud[i]) - x0;
            float D = std::fabs(nrm.dot(w));
            if(D < p_to_plane_thresh)
              ++num_inliers;
        }
        float ratio = float(num_inliers) / float(corner_cloud.size());
        if (ratio > best_ratio) {
          best_ratio = ratio;
          n = nrm;
          d = -x0.dot(nrm);
        }
      }

      if(best_ratio < 0.9)
      {
        ROS_ERROR ("Could not estimate a planar model from the checkboard corners.");
        last_sample_invalid_ = true;
        return;
      }

      ROS_DEBUG_STREAM( "Model coefficients: " << n[0] << " "
                                          << n[1] << " "
                                          << n[2] << " "
                                          << d );

      unsigned vi=0;
      for(size_t i = 0; i< features.image_points.size(); i++)
      {
        // intersect pixel ray with plane
        geometry_msgs::Point& pixel = features.image_points[i];

        cv::Point3f ray_pt;
        cloud_converter_.depthTo3DPoint( pixel, 1.0, ray_pt );
        cv::Vec3f ray(ray_pt);
        ray = ray / cv::norm(ray);

        float d1 = ray.dot(n);
        float lambda = d / d1;

        ray = ray * lambda;

        if ( ray[2] <  0 )
        {
          ray = -ray;
        }

        ///////////////////////////////
        /*
        float depth = *(depth_ptr+width*(unsigned int)pixel.y+(unsigned int)pixel.x);
        if ( !isnan(depth) )
        {
          pcl::PointXYZ point;
          cloud_converter_.depthTo3DPoint( pixel, depth, point );
          std::cout << "3d point: " << point.x << ", " << point.y << ", " << point.z << std::endl;
        }
        std::cout << "ray-plane intersection: " << ray.x << ", " << ray.y << ", " << ray.z << std::endl;
        */
        /////////////////////////////////

        pixel.z = ray[2];
      }

      // print 'back to normal' message once
      if ( last_sample_invalid_ )
      {
        ROS_INFO_STREAM(s.str() << "Successfuly detected checkerboard.");
      }

      last_sample_invalid_ = false;
      pub_.publish(features);
    }
  }

private:
  boost::mutex run_mutex_;
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<image_cb_detector::ConfigAction> as_;
  image_cb_detector::ImageCbDetector detector_;

  message_filters::Subscriber<sensor_msgs::Image> image_sub_; 
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;
  message_filters::Synchronizer<CameraSyncPolicy> sync_;

  ros::Publisher pub_;

  bool last_sample_invalid_;
  DepthToPointCloud cloud_converter_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_cb_detector_action");
  ros::NodeHandle n;
  RgbdCbDetectorAction detector_action(n);
  ros::spin();
  return 0;
}

