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

#include <ros/console.h>
#include <laser_cb_detector/cv_laser_bridge.h>

using namespace laser_cb_detector;

bool CvLaserBridge::fromIntensity(const calibration_msgs::DenseLaserSnapshot& snapshot, float min_val, float max_val)
{
  if (snapshot.num_scans * snapshot.readings_per_scan != snapshot.intensities.size())
  {
    ROS_ERROR("Got malformed snapshot. Expected [%u x %u]=%u, but snapshot.intensities.size()=%u",
              snapshot.num_scans, snapshot.readings_per_scan,
              snapshot.num_scans * snapshot.readings_per_scan,
              snapshot.intensities.size());
    return false;
  }

  fromSnapshot(snapshot, snapshot.intensities, min_val, max_val);
  return true;
}

bool CvLaserBridge::reallocIfNeeded(IplImage** img, CvSize sz)
{
  int depth = IPL_DEPTH_8U;
  int channels = 1;

  if ((*img) != 0)
  {
    if ((*img)->width     != sz.width  ||
        (*img)->height    != sz.height ||
        (*img)->depth     != depth     ||
        (*img)->nChannels != channels)
    {
      cvReleaseImage(img);
      *img = 0;
    }
  }

  if (*img == 0)
  {
    *img = cvCreateImage(sz, depth, channels);
    return true;
  }
  return false;
}

void CvLaserBridge::fromSnapshot(const calibration_msgs::DenseLaserSnapshot& snapshot, const std::vector<float>& src, float min_val, float max_val)
{
  assert(snapshot.num_scans * snapshot.readings_per_scan == src.size());

  CvMat cvmHeader;

  IplImage source_image;

  cvInitMatHeader(&cvmHeader, snapshot.num_scans, snapshot.readings_per_scan, CV_32FC1, const_cast<float*>(&(src[0])));
  cvGetImage(&cvmHeader, &source_image);

  reallocIfNeeded(&dest_image_, cvSize(source_image.width, source_image.height) );

  double range = (max_val - min_val);
  double scale = 255/range;
  double shift = -min_val * 255/range;

  ROS_DEBUG("Scale: %f   Shift: %f\n", scale, shift);

  cvConvertScale(&source_image, dest_image_, scale, shift);
}
