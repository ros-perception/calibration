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
#include <boost/thread/mutex.hpp>

#include <calibration_msgs/CalibrationPattern.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

namespace image_cb_detector
{

class ImageAnnotator
{
public:
  ImageAnnotator();

  void processPair(const sensor_msgs::ImageConstPtr& image, const calibration_msgs::CalibrationPatternConstPtr& features);
private:
  ros::Publisher image_pub_;
  ros::NodeHandle n_;

  // Params
  int marker_size_;
  double scaling_;
};

}

using namespace image_cb_detector;

ImageAnnotator::ImageAnnotator()
{
  image_pub_ = n_.advertise<sensor_msgs::Image>("annotated", 1);

  ros::NodeHandle local_ns_("~");

  local_ns_.param("marker_size", marker_size_, 1);
  local_ns_.param("scaling", scaling_, 1.0);
  ROS_INFO("[marker_size]: %i", marker_size_);
  ROS_INFO("[scaling]: %.3f", scaling_);
}

void ImageAnnotator::processPair(const sensor_msgs::ImageConstPtr& image, const calibration_msgs::CalibrationPatternConstPtr& features)
{
  try {
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image, "rgb8");

    // ***** Resize the image based on scaling parameters in config *****
    const int scaled_width  = (int) (.5 + cv_image->image.cols  * scaling_);
    const int scaled_height = (int) (.5 + cv_image->image.rows * scaling_);
    cv::Mat cv_image_scaled;
    cv::resize(cv_image->image, cv_image_scaled, 
          cv::Size(scaled_width, scaled_height), 0, 0, CV_INTER_LINEAR);

    if (features->success)
    {
      cv::Point2i pt0(features->image_points[0].x*scaling_, 
            features->image_points[0].y*scaling_);
      cv::circle(cv_image_scaled, pt0, marker_size_*2, cvScalar(0,0,255), 1) ;
      for (unsigned int i=0; i<features->image_points.size(); i++)
      {
        cv::Point2i pt(features->image_points[i].x*scaling_, 
              features->image_points[i].y*scaling_);
        cv::circle(cv_image_scaled, pt, marker_size_, cvScalar(0,255,0), 1) ;
      }
    }
    else
    {
      for (unsigned int i=0; i<features->image_points.size(); i++)
      {
        cv::Point2i pt(features->image_points[i].x*scaling_,
              features->image_points[i].y*scaling_);
        cv::circle(cv_image_scaled, pt, marker_size_, cvScalar(255,0,0), 1) ;
      }
    }

    // Send the annotated image over ROS
    sensor_msgs::Image result_image = *(cv_bridge::CvImage(cv_image->header, cv_image->encoding, cv_image_scaled).toImageMsg());
    image_pub_.publish(result_image);
  } catch(cv_bridge::Exception & e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_annotator");

  ros::NodeHandle nh;

  ImageAnnotator annotator;

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<calibration_msgs::CalibrationPattern> features_sub(nh, "features", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image,
                                    calibration_msgs::CalibrationPattern> sync(image_sub, features_sub, 5);

  sync.registerCallback(boost::bind(&ImageAnnotator::processPair, &annotator, _1, _2));

  ros::spin();
  return 0;
}
