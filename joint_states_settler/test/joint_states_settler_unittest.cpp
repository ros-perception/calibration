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

#include <joint_states_settler/ConfigGoal.h>
#include <joint_states_settler/joint_states_settler.h>

using namespace std;
using namespace joint_states_settler;

// Number of samples
static const unsigned int N = 9;

// Number of channels/joints
static const unsigned int C = 3;

// List of joint names
static const char* names[C] = { "A", "B", "C" };

// List of joint positions
static const double data[N][C] = { { 0,  0, 10},
                                   { 1,  1, 15},
                                   { 2,  2, 20},
                                   { 3,  3, 50},
                                   { 4,  4, 30},
                                   { 3,  5, 35},
                                   { 2,  6, 25},
                                   { 1,  7, 20},
                                   { 0,  8, 25 } };

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

// add data to the settler, and see what intervals come out
vector<calibration_msgs::Interval> addToSettler(JointStatesSettler& settler, unsigned int time_channel)
{
  vector<calibration_msgs::Interval> intervals;

  for (unsigned int i=0; i<N; i++)
  {
    sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState);
    msg->header.stamp = ros::Time(times[i][time_channel], 0);

    msg->name.resize(C);
    msg->position.resize(C);
    msg->velocity.resize(C);
    msg->effort.resize(C);

    for (unsigned int j=0; j<C; j++)
    {
      msg->name[j] = names[j];
      msg->position[j] = data[i][j];
    }

    intervals.push_back(settler.add(msg));
  }

  return intervals;
}

// Tight tol on A, and loose on C
ConfigGoal config1()
{
  ConfigGoal config;
  config.joint_names.clear();
  config.joint_names.push_back("A");
  config.joint_names.push_back("C");

  config.tolerances.clear();
  config.tolerances.push_back(2.5);
  config.tolerances.push_back(100);

  config.max_step = ros::Duration(2,0);

  config.cache_size = 100;

  return config;
}

// All loose tolerances
ConfigGoal config2()
{
  ConfigGoal config;
  config.joint_names.clear();
  config.joint_names.push_back("A");
  config.joint_names.push_back("B");
  config.joint_names.push_back("C");

  config.tolerances.clear();
  config.tolerances.push_back(100);
  config.tolerances.push_back(100);
  config.tolerances.push_back(100);

  config.max_step = ros::Duration(2,0);

  config.cache_size = 100;

  return config;
}

void doEasyCheck(const vector<calibration_msgs::Interval>& intervals)
{
  ASSERT_EQ(intervals.size(), N);
  EXPECT_EQ(intervals[0].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[0].end.sec,   (unsigned int) 0);
  EXPECT_EQ(intervals[1].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[1].end.sec,   (unsigned int) 1);
  EXPECT_EQ(intervals[2].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[2].end.sec,   (unsigned int) 2);
  EXPECT_EQ(intervals[3].start.sec, (unsigned int) 1);
  EXPECT_EQ(intervals[3].end.sec,   (unsigned int) 3);
  EXPECT_EQ(intervals[4].start.sec, (unsigned int) 2);
  EXPECT_EQ(intervals[4].end.sec,   (unsigned int) 4);
  EXPECT_EQ(intervals[5].start.sec, (unsigned int) 2);
  EXPECT_EQ(intervals[5].end.sec,   (unsigned int) 5);
  EXPECT_EQ(intervals[6].start.sec, (unsigned int) 2);
  EXPECT_EQ(intervals[6].end.sec,   (unsigned int) 6);
  EXPECT_EQ(intervals[7].start.sec, (unsigned int) 5);
  EXPECT_EQ(intervals[7].end.sec,   (unsigned int) 7);
  EXPECT_EQ(intervals[8].start.sec, (unsigned int) 6);
  EXPECT_EQ(intervals[8].end.sec,   (unsigned int) 8);
}

void doMaxStepCheck(const vector<calibration_msgs::Interval>& intervals)
{
  ASSERT_EQ(intervals.size(), N);
  EXPECT_EQ(intervals[0].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[0].end.sec,   (unsigned int) 0);
  EXPECT_EQ(intervals[1].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[1].end.sec,   (unsigned int) 1);
  EXPECT_EQ(intervals[2].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[2].end.sec,   (unsigned int) 2);
  EXPECT_EQ(intervals[3].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[3].end.sec,   (unsigned int) 3);
  EXPECT_EQ(intervals[4].start.sec, (unsigned int) 0);
  EXPECT_EQ(intervals[4].end.sec,   (unsigned int) 4);
  EXPECT_EQ(intervals[5].start.sec, (unsigned int) 15);
  EXPECT_EQ(intervals[5].end.sec,   (unsigned int) 15);
  EXPECT_EQ(intervals[6].start.sec, (unsigned int) 15);
  EXPECT_EQ(intervals[6].end.sec,   (unsigned int) 16);
  EXPECT_EQ(intervals[7].start.sec, (unsigned int) 15);
  EXPECT_EQ(intervals[7].end.sec,   (unsigned int) 17);
  EXPECT_EQ(intervals[8].start.sec, (unsigned int) 15);
  EXPECT_EQ(intervals[8].end.sec,   (unsigned int) 18);
}


TEST(JointStatesSettler, easyCheck)
{
  JointStatesSettler settler;

  bool config_result = settler.configure(config1());
  ASSERT_TRUE(config_result);

  vector<calibration_msgs::Interval> intervals = addToSettler(settler, 0);

  doEasyCheck(intervals);
}

TEST(JointStatesSettler, maxStepCheck)
{
  JointStatesSettler settler;

  bool config_result = settler.configure(config2());
  ASSERT_TRUE(config_result);

  vector<calibration_msgs::Interval> intervals = addToSettler(settler, 1);

  doMaxStepCheck(intervals);
}

TEST(JointStatesSettler, reconfigureCheck)
{
  JointStatesSettler settler;

  bool config_result = settler.configure(config1());
  ASSERT_TRUE(config_result);

  vector<calibration_msgs::Interval> intervals = addToSettler(settler, 0);

  doEasyCheck(intervals);

  config_result = settler.configure(config2());
  ASSERT_TRUE(config_result);

  intervals = addToSettler(settler, 1);

  doMaxStepCheck(intervals);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
