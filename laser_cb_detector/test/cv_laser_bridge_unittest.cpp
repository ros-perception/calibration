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

#include <gtest/gtest.h>


#include <calibration_msgs/DenseLaserSnapshot.h>
#include <laser_cb_detector/cv_laser_bridge.h>

using namespace std;
using namespace laser_cb_detector;

static const bool DEBUG=false;

calibration_msgs::DenseLaserSnapshot buildSnapshot(double start_range, double start_intensity, double increment,
                                                   const unsigned int H, const unsigned int W)
{
  calibration_msgs::DenseLaserSnapshot snapshot;

  snapshot.readings_per_scan = W;
  snapshot.num_scans = H;

  snapshot.ranges.resize(H*W);
  snapshot.intensities.resize(H*W);

  for (unsigned int i=0; i<H*W; i++)
  {
    snapshot.ranges[i]      = start_range     + i*increment;
    snapshot.intensities[i] = start_intensity + i*increment;
  }

  return snapshot;
}

static const unsigned int NUM_SCANS = 5;
static const unsigned int RAYS_PER_SCAN = 10;
static const double eps = 1e-8;

void displayImage(IplImage* image)
{
  for (int i=0; i<image->height; i++)
  {
    for (int j=0; j<image->width; j++)
    {
      printf("%3u  ", *((unsigned char*)(image->imageData)+ i*image->widthStep + j));
    }
    printf("\n");
  }
}

TEST(CvLaserBridge, easy)
{
  CvLaserBridge bridge;

  calibration_msgs::DenseLaserSnapshot snapshot;

  snapshot = buildSnapshot(100.0, 0.0, 1.0, NUM_SCANS, RAYS_PER_SCAN);

  bool result;

  result = bridge.fromIntensity(snapshot, 0, 49);
  IplImage* image = bridge.toIpl();

  ASSERT_TRUE(result);
  ASSERT_TRUE(image);

  EXPECT_EQ(image->height, (int) NUM_SCANS);
  EXPECT_EQ(image->width,  (int) RAYS_PER_SCAN);

  if (DEBUG)
    displayImage(image);

  // Check the first and last pixel in the image
  EXPECT_EQ( (int) *((unsigned char*)(image->imageData)+0), 0);
  EXPECT_EQ( (int) *((unsigned char*)(image->imageData)+4*image->widthStep + 9), 255);
}

TEST(CvLaserBridge, saturationTest)
{
  CvLaserBridge bridge;
  calibration_msgs::DenseLaserSnapshot snapshot;
  snapshot = buildSnapshot(100.0, 0.0, 1.0, NUM_SCANS, RAYS_PER_SCAN);

  bool result;
  result = bridge.fromIntensity(snapshot, 10, 20);
  IplImage* image = bridge.toIpl();
  ASSERT_TRUE(result);
  ASSERT_TRUE(image);

  EXPECT_EQ(image->height, (int) NUM_SCANS);
  EXPECT_EQ(image->width,  (int) RAYS_PER_SCAN);

  if (DEBUG)
    displayImage(image);

  // Check to make sure some of the pixels saturated near the beginning and end of the range
  EXPECT_EQ( (int) *((unsigned char*)(image->imageData)+5), 0);
  EXPECT_EQ( (int) *((unsigned char*)(image->imageData)+45), 255);
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
