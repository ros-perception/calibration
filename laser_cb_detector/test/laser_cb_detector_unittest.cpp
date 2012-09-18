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


#include <laser_cb_detector/ConfigGoal.h>
#include <laser_cb_detector/laser_cb_detector.h>
#include <opencv/highgui.h>

using namespace laser_cb_detector;
using namespace std;

static const double pix_eps_ = .6 ;
static const double eps = 1e-6;

static const bool DEBUG=false;


calibration_msgs::DenseLaserSnapshot getSnapshot(const string& filename)
{
  IplImage* image;
  image = cvLoadImage(filename.c_str(), 0);         // 0 -> Force image to grayscale
  EXPECT_TRUE(image) << "could not open image file [" << filename << "]" << endl;


  IplImage* float_image;
  float_image = cvCreateImage( cvSize(image->width, image->height), IPL_DEPTH_32F, 1);
  cvConvert(image, float_image);

  calibration_msgs::DenseLaserSnapshot snapshot;
  snapshot.readings_per_scan = image->width;
  snapshot.num_scans = image->height;
  snapshot.intensities.resize(image->height * image->width);

  for (int i=0; i<float_image->height; i++)
  {
    memcpy(&snapshot.intensities[i*snapshot.readings_per_scan],
           (float_image->imageData + i*float_image->widthStep),
           sizeof(float)*snapshot.readings_per_scan);
  }

  if (DEBUG)
  {
    for (unsigned int i=0; i<snapshot.num_scans; i++)
    {
      for (unsigned int j=0; j<snapshot.readings_per_scan; j++)
      {
        if (snapshot.intensities[i*snapshot.readings_per_scan + j] < 100.0)
          printf(" ");
        else
          printf("X");
      }
      printf("\n");
    }
  }

  cvReleaseImage(&float_image);
  cvReleaseImage(&image);

  return snapshot;
}

ConfigGoal config3x4()
{
  ConfigGoal config;

  config.num_x = 3;
  config.num_y = 4;
  config.spacing_x = 1.0;
  config.spacing_y = 1.0;

  config.width_scaling = 1.0;
  config.height_scaling = 1.0;

  config.min_intensity = 0.0;
  config.max_intensity = 100.0;

  config.subpixel_window = 3;
  config.subpixel_zero_zone = 1;

  config.flip_horizontal = 0;

  return config;
}


void check3x4(const calibration_msgs::CalibrationPattern& result, double pix_eps, bool flipped)
{
  EXPECT_TRUE(result.success);


  EXPECT_NEAR(result.object_points[0].x, 0.0, eps);
  EXPECT_NEAR(result.object_points[0].y, 0.0, eps);
  EXPECT_NEAR(result.object_points[0].z, 0.0, eps);

  EXPECT_NEAR(result.object_points[2].x, 2.0, eps);
  EXPECT_NEAR(result.object_points[2].y, 0.0, eps);
  EXPECT_NEAR(result.object_points[2].z, 0.0, eps);

  EXPECT_NEAR(result.object_points[5].x, 2.0, eps);
  EXPECT_NEAR(result.object_points[5].y, 1.0, eps);
  EXPECT_NEAR(result.object_points[5].z, 0.0, eps);

  // Detected corners
  if (flipped)
  {
    EXPECT_NEAR(result.image_points[0].x,  81.5, pix_eps);
    EXPECT_NEAR(result.image_points[0].y,  72.0, pix_eps);

    EXPECT_NEAR(result.image_points[1].x,  81.5, pix_eps);
    EXPECT_NEAR(result.image_points[1].y, 118.5, pix_eps);

    EXPECT_NEAR(result.image_points[11].x,  223.5, pix_eps);
    EXPECT_NEAR(result.image_points[11].y,  166.5, pix_eps);
  }
  else
  {
    EXPECT_NEAR(result.image_points[0].x, 223.0, pix_eps);
    EXPECT_NEAR(result.image_points[0].y,  72.0, pix_eps);

    EXPECT_NEAR(result.image_points[1].x, 223.0, pix_eps);
    EXPECT_NEAR(result.image_points[1].y, 119.0, pix_eps);

    EXPECT_NEAR(result.image_points[11].x,  81.0, pix_eps);
    EXPECT_NEAR(result.image_points[11].y, 166.0, pix_eps);
  }
}

TEST(LaserCbDetector, easy_cb_3x4)
{
  LaserCbDetector detector;
  calibration_msgs::DenseLaserSnapshot snapshot;

  snapshot = getSnapshot("test/data/cb_3x4.png");

  ASSERT_EQ(snapshot.readings_per_scan, (unsigned int) 303);
  ASSERT_EQ(snapshot.num_scans, (unsigned int) 325);

  detector.configure(config3x4());

  bool return_result;
  calibration_msgs::CalibrationPattern result;
  return_result = detector.detect(snapshot, result);
  EXPECT_TRUE(return_result);

  check3x4(result, pix_eps_, false);
}

TEST(LaserCbDetector, scaled_cb_3x4)
{
  LaserCbDetector detector;
  calibration_msgs::DenseLaserSnapshot snapshot;

  snapshot = getSnapshot("test/data/cb_3x4.png");

  ASSERT_EQ(snapshot.readings_per_scan, (unsigned int) 303);
  ASSERT_EQ(snapshot.num_scans, (unsigned int) 325);

  ConfigGoal config = config3x4();
  config.width_scaling  = 2.0;
  config.height_scaling = 4.0;
  detector.configure(config);

  bool return_result;
  calibration_msgs::CalibrationPattern result;
  return_result = detector.detect(snapshot, result);
  EXPECT_TRUE(return_result);

  //! \todo Should I be worried that I need a bigger eps when there's scaling? Seems a little shady
  check3x4(result, 1.0, false);
}

TEST(LaserCbDetector, reflected_cb_3x4)
{
  LaserCbDetector detector;
  calibration_msgs::DenseLaserSnapshot snapshot;

  snapshot = getSnapshot("test/data/cb_3x4.png");

  ASSERT_EQ(snapshot.readings_per_scan, (unsigned int) 303);
  ASSERT_EQ(snapshot.num_scans, (unsigned int) 325);

  ConfigGoal config = config3x4();
  config.flip_horizontal = 1;
  detector.configure(config);

  bool return_result;
  calibration_msgs::CalibrationPattern result;
  return_result = detector.detect(snapshot, result);
  EXPECT_TRUE(return_result);

  //! \todo Should I be worried that I need a bigger eps when there's scaling? Seems a little shady
  check3x4(result, 1.0, true);
}


TEST(LaserCbDetector, malformedTest)
{
  LaserCbDetector detector;

  calibration_msgs::DenseLaserSnapshot snapshot;

  detector.configure(config3x4());
  snapshot = getSnapshot("test/data/cb_3x4.png");
  snapshot.num_scans = 10;

  bool return_result;
  calibration_msgs::CalibrationPattern result;
  return_result = detector.detect(snapshot, result);

  EXPECT_FALSE(return_result);
}

TEST(LaserCbDetector, noCheckerboardTest)
{
  LaserCbDetector detector;

  calibration_msgs::DenseLaserSnapshot snapshot;

  ConfigGoal config = config3x4();
  config.num_x = 4;
  detector.configure(config);
  snapshot = getSnapshot("test/data/cb_3x4.png");

  bool return_result;
  calibration_msgs::CalibrationPattern result;
  return_result = detector.detect(snapshot, result);

  EXPECT_TRUE(return_result);
  EXPECT_FALSE(result.success);
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
