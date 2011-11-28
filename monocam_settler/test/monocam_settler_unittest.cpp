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

#include <monocam_settler/ConfigGoal.h>
#include <monocam_settler/monocam_settler.h>

using namespace std;
using namespace monocam_settler;



static const unsigned int N = 9;
static const unsigned int C = 2;

static const double data[N][2*C] = { {  0, 0, 0, 0 },
                                     {  1, 2, 1, 1 },
                                     {  2, 4, 0, 0 },
                                     {  3, 6, 1, 1 },
                                     {  4, 8, 0, 0 },
                                     {  0, 0, 1, 1 },
                                     {  4, 1, 0, 0 },
                                     {  8, 2, 1, 1 },
                                     { 12, 3, 0, 0 }};

// 2 possible sets of timestamps for the data
static const unsigned int times[N][2] = { { 0,  0 },
                                          { 1,  1 },
                                          { 2,  2 },
                                          { 3,  3 },
                                          { 4,  4 },
                                          { 5, 15 },
                                          { 6, 16 },
                                          { 7, 17 },
                                          { 8, 18 } };

static const bool success[N] = { true,
                                 true,
                                 false,
                                 true,
                                 true,
                                 true,
                                 true,
                                 true,
                                 true };

// add data to the settler, and see what intervals come out
vector<calibration_msgs::Interval> addToSettler(MonocamSettler& settler, unsigned int time_channel, bool always_success)
{
  vector<calibration_msgs::Interval> intervals;

  for (unsigned int i=0; i<N; i++)
  {
    calibration_msgs::CalibrationPatternPtr msg(new calibration_msgs::CalibrationPattern);
    msg->header.stamp = ros::Time(times[i][time_channel], 0);

    msg->image_points.resize(C);
    for (unsigned int j=0; j<C; j++)
    {
      msg->image_points[j].x = data[i][2*j+0];
      msg->image_points[j].y = data[i][2*j+1];
    }
    msg->success = always_success || success[i];

    calibration_msgs::Interval cur_interval;
    bool result = settler.add(msg, cur_interval);
    if (result)
      intervals.push_back(cur_interval);
    else
      intervals.push_back(calibration_msgs::Interval());
  }

  return intervals;
}

void doEasyCheck(const vector<calibration_msgs::Interval>& intervals)
{
  ASSERT_EQ(intervals.size(), N);
  EXPECT_EQ(intervals[0].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[0].end.sec,   (unsigned int) 0);

  EXPECT_EQ(intervals[1].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[1].end.sec,   (unsigned int) 1);

  EXPECT_EQ(intervals[2].start.sec, (unsigned int) 1);
  EXPECT_EQ(intervals[2].end.sec,   (unsigned int) 2);

  EXPECT_EQ(intervals[3].start.sec, (unsigned int) 2);
  EXPECT_EQ(intervals[3].end.sec,   (unsigned int) 3);

  EXPECT_EQ(intervals[4].start.sec, (unsigned int) 3);
  EXPECT_EQ(intervals[4].end.sec,   (unsigned int) 4);

  EXPECT_EQ(intervals[5].start.sec, (unsigned int) 5);
  EXPECT_EQ(intervals[5].end.sec,   (unsigned int) 5);

  EXPECT_EQ(intervals[6].start.sec, (unsigned int) 6);
  EXPECT_EQ(intervals[6].end.sec,   (unsigned int) 6);

  EXPECT_EQ(intervals[7].start.sec, (unsigned int) 7);
  EXPECT_EQ(intervals[7].end.sec,   (unsigned int) 7);

  EXPECT_EQ(intervals[8].start.sec, (unsigned int) 8);
  EXPECT_EQ(intervals[8].end.sec,   (unsigned int) 8);
}

void doIgnoreFailuresCheck(const vector<calibration_msgs::Interval>& intervals)
{
  ASSERT_EQ(intervals.size(), N);
  EXPECT_EQ(intervals[0].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[0].end.sec,   (unsigned int) 0);

  EXPECT_EQ(intervals[1].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[1].end.sec,   (unsigned int) 1);

  EXPECT_EQ(intervals[2].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[2].end.sec,   (unsigned int) 0);

  EXPECT_EQ(intervals[3].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[3].end.sec,   (unsigned int) 3);

  EXPECT_EQ(intervals[4].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[4].end.sec,   (unsigned int) 4);

  EXPECT_EQ(intervals[5].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[5].end.sec,   (unsigned int) 5);

  EXPECT_EQ(intervals[6].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[6].end.sec,   (unsigned int) 6);

  EXPECT_EQ(intervals[7].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[7].end.sec,   (unsigned int) 7);

  EXPECT_EQ(intervals[8].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[8].end.sec,   (unsigned int) 8);
}

void doCatchFailuresCheck(const vector<calibration_msgs::Interval>& intervals)
{
  ASSERT_EQ(intervals.size(), N);
  EXPECT_EQ(intervals[0].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[0].end.sec,   (unsigned int) 0);

  EXPECT_EQ(intervals[1].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[1].end.sec,   (unsigned int) 1);

  EXPECT_EQ(intervals[2].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[2].end.sec,   (unsigned int) 0);

  EXPECT_EQ(intervals[3].start.sec, (unsigned int) 3);
  EXPECT_EQ(intervals[3].end.sec,   (unsigned int) 3);

  EXPECT_EQ(intervals[4].start.sec, (unsigned int) 3);
  EXPECT_EQ(intervals[4].end.sec,   (unsigned int) 4);

  EXPECT_EQ(intervals[5].start.sec, (unsigned int) 3);
  EXPECT_EQ(intervals[5].end.sec,   (unsigned int) 5);

  EXPECT_EQ(intervals[6].start.sec, (unsigned int) 3);
  EXPECT_EQ(intervals[6].end.sec,   (unsigned int) 6);

  EXPECT_EQ(intervals[7].start.sec, (unsigned int) 3);
  EXPECT_EQ(intervals[7].end.sec,   (unsigned int) 7);

  EXPECT_EQ(intervals[8].start.sec, (unsigned int) 3);
  EXPECT_EQ(intervals[8].end.sec,   (unsigned int) 8);
}

ConfigGoal config1()
{
  ConfigGoal config;

  config.tolerance = 2.5;

  config.ignore_failures = true;

  config.max_step = ros::Duration(2,0);

  config.cache_size = 100;

  return config;
}

ConfigGoal config2(bool ignore_failures)
{
  ConfigGoal config;

  config.tolerance = 100.0;

  config.ignore_failures = ignore_failures;

  config.max_step = ros::Duration(2,0);

  config.cache_size = 100;

  return config;
}

TEST(MonocamSettler, easyCheck)
{
  MonocamSettler settler;

  bool config_result = settler.configure(config1());
  ASSERT_TRUE(config_result);

  vector<calibration_msgs::Interval> intervals = addToSettler(settler, 0, true);

  doEasyCheck(intervals);
}

TEST(MonocamSettler, ignoreFailuresCheck)
{
  MonocamSettler settler;

  bool config_result = settler.configure(config2(true));
  ASSERT_TRUE(config_result);

  vector<calibration_msgs::Interval> intervals = addToSettler(settler, 0, false);

  doIgnoreFailuresCheck(intervals);
}

TEST(MonocamSettler, catchFailuresCheck)
{
  MonocamSettler settler;

  bool config_result = settler.configure(config2(false));
  ASSERT_TRUE(config_result);

  vector<calibration_msgs::Interval> intervals = addToSettler(settler, 0, false);

  doCatchFailuresCheck(intervals);
}

TEST(MonocamSettler, reconfigureCheck)
{
  MonocamSettler settler;

  bool config_result = settler.configure(config1());
  ASSERT_TRUE(config_result);

  vector<calibration_msgs::Interval> intervals = addToSettler(settler, 0, true);

  doEasyCheck(intervals);

  config_result = settler.configure(config2(false));
  ASSERT_TRUE(config_result);

  intervals = addToSettler(settler, 0, false);

  doCatchFailuresCheck(intervals);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
