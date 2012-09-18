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

#include <laser_cb_detector/laser_interval_calc.h>

using namespace std;
using namespace laser_cb_detector;

static const bool DEBUG=false;

static const float eps = 1e-6;

class LaserIntervalCalcTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    snapshot_.header.stamp = ros::Time(10,0);
    snapshot_.time_increment = 1.0;
    snapshot_.readings_per_scan = 4;
    snapshot_.num_scans = 3;
    snapshot_.scan_start.resize(3);
    snapshot_.scan_start[0] = ros::Time(20);
    snapshot_.scan_start[1] = ros::Time(40);
    snapshot_.scan_start[2] = ros::Time(30);

    // Snapshot Times:
    // 20  21  22  23  24
    // 40  41  42  43  44
    // 30  31  32  33  34
  }

  calibration_msgs::DenseLaserSnapshot snapshot_;
  calibration_msgs::CalibrationPattern features_;
  calibration_msgs::Interval interval_;
};

#define EXPECT_ROSTIME_NEAR(a, b) \
  EXPECT_NEAR( (a-b).toSec(), 0.0, eps) \
     << "Times don't match: #a: " << a.toSec() << "   " << "#b: " << b.toSec()

TEST_F(LaserIntervalCalcTest, easy_single_point)
{
  bool success;
  features_.image_points.resize(1);
  features_.image_points[0].x = .5;
  features_.image_points[0].y = 1.5;
  success = LaserIntervalCalc::computeInterval(snapshot_, features_, interval_);
  EXPECT_TRUE(success);
  EXPECT_ROSTIME_NEAR(interval_.start, ros::Time(30,0));
  EXPECT_ROSTIME_NEAR(interval_.end,   ros::Time(41,0));
}

TEST_F(LaserIntervalCalcTest, corners)
{
  bool success;
  features_.image_points.resize(1);
  features_.image_points[0].x =  .5;
  features_.image_points[0].y = 0.01;
  success = LaserIntervalCalc::computeInterval(snapshot_, features_, interval_);
  EXPECT_TRUE(success);
  EXPECT_ROSTIME_NEAR(interval_.start, ros::Time(20,0));
  EXPECT_ROSTIME_NEAR(interval_.end,   ros::Time(41,0));

  features_.image_points[0].y = -0.01;
  success = LaserIntervalCalc::computeInterval(snapshot_, features_, interval_);
  EXPECT_FALSE(success);

  features_.image_points[0].y = 2.01;
  success = LaserIntervalCalc::computeInterval(snapshot_, features_, interval_);
  EXPECT_FALSE(success);
}

TEST_F(LaserIntervalCalcTest, empty)
{
  bool success;
  features_.image_points.clear();
  success = LaserIntervalCalc::computeInterval(snapshot_, features_, interval_);
  EXPECT_TRUE(success);
  EXPECT_ROSTIME_NEAR(interval_.start, ros::Time(10,0));
  EXPECT_ROSTIME_NEAR(interval_.end,   ros::Time(10,0));
}

TEST_F(LaserIntervalCalcTest, easy_multi_point)
{
  bool success;
  features_.image_points.resize(3);
  features_.image_points[0].x =  .5;
  features_.image_points[0].y =  .5;
  features_.image_points[1].x = 3.5;
  features_.image_points[1].y =  .5;
  features_.image_points[2].x = 3.5;
  features_.image_points[2].y = 1.5;

  success = LaserIntervalCalc::computeInterval(snapshot_, features_, interval_);
  EXPECT_TRUE(success);
  EXPECT_ROSTIME_NEAR(interval_.start, ros::Time(20,0));
  EXPECT_ROSTIME_NEAR(interval_.end,   ros::Time(44,0));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
