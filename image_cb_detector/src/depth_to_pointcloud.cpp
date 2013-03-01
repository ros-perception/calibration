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


#include "image_cb_detector/depth_to_pointcloud.h"



DepthToPointCloud::DepthToPointCloud()
{
  // TODO Auto-generated constructor stub

}

DepthToPointCloud::~DepthToPointCloud()
{
  // TODO Auto-generated destructor stub
}

void DepthToPointCloud::initialize(sensor_msgs::ImageConstPtr depth_msg,
                                   sensor_msgs::CameraInfoConstPtr camera_info_msg)
{
  if (!depth_msg || !camera_info_msg)
  {
    return;
  }

  std::size_t width = depth_msg->width;
  std::size_t height = depth_msg->height;

  if (width != projection_map_x_.size() || height !=projection_map_y_.size())
  {
    // Procompute 3D projection matrix
    //
    // The following computation of center_x,y and fx,fy duplicates
    // code in the image_geometry package, but this avoids dependency
    // on OpenCV, which simplifies releasing rviz.

    // Use correct principal point from calibration
    float center_x = camera_info_msg->P[2] - camera_info_msg->roi.x_offset;
    float center_y = camera_info_msg->P[6] - camera_info_msg->roi.y_offset;

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double scale_x = camera_info_msg->binning_x > 1 ? (1.0 / camera_info_msg->binning_x) : 1.0;
    double scale_y = camera_info_msg->binning_y > 1 ? (1.0 / camera_info_msg->binning_y) : 1.0;

    double fx = camera_info_msg->P[0] * scale_x;
    double fy = camera_info_msg->P[5] * scale_y;

    float constant_x = 1.0f / fx;
    float constant_y = 1.0f / fy;

    projection_map_x_.resize(width);
    projection_map_y_.resize(height);
    std::vector<float>::iterator projX = projection_map_x_.begin();
    std::vector<float>::iterator projY = projection_map_y_.begin();

    // precompute 3D projection matrix
    for (int v = 0; v < height; ++v, ++projY)
      *projY = (v - center_y) * constant_y;

    for (int u = 0; u < width; ++u, ++projX)
      *projX = (u - center_x) * constant_x;
  }
}

