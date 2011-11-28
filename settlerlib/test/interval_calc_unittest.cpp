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
#include <settlerlib/interval_calc.h>
#include <settlerlib/sorted_deque.h>

using namespace std;
using namespace settlerlib;


const unsigned int NA = 9;
static const double dataA[NA][2] = { { 0,  0},
                                     { 1,  1},
                                     { 2,  2},
                                     { 3,  3},
                                     { 4,  4},
                                     { 3,  5},
                                     { 2,  6},
                                     { 1,  7},
                                     { 0,  8} };

/**
 * Generate data for 2 channels
 */
SortedDeque<DeflatedConstPtr> generateSignal1()
{
  SortedDeque<DeflatedConstPtr> signal(&SortedDeque<DeflatedConstPtr>::getPtrStamp) ;
  signal.setMaxSize(20);

  for (unsigned int i=0; i<NA; i++)
  {
    DeflatedPtr deflated(new Deflated);
    deflated->header.stamp = ros::Time(i,0);
    deflated->channels_.resize(2);
    deflated->channels_[0] = dataA[i][0];
    deflated->channels_[1] = dataA[i][1];

    signal.add(deflated);
  }

  return signal;
}

/**
 * Generate data for 2 channels. Same at generateSignal1, except there's a time discontinuity between samples 4 & 5.
 */
SortedDeque<DeflatedConstPtr> generateSignal2()
{

  SortedDeque<DeflatedConstPtr> signal(&SortedDeque<DeflatedConstPtr>::getPtrStamp) ;
  signal.setMaxSize(20);

  for (unsigned int i=0; i<NA; i++)
  {
    DeflatedPtr deflated(new Deflated);
    deflated->header.stamp = ros::Time(i,0);
    if (i > 4)
      deflated->header.stamp = ros::Time(i+10,0);
    else
      deflated->header.stamp = ros::Time(i,0);
    deflated->channels_.resize(2);
    deflated->channels_[0] = dataA[i][0];
    deflated->channels_[1] = dataA[i][1];

    signal.add(deflated);
  }

  return signal;
}

// A pretty simple test where the first channel exceeds the tolerance first
TEST(IntervalCalc, easy1)
{
  SortedDeque<DeflatedConstPtr> signal = generateSignal1();

  vector<double> tol(2);
  tol[0] = 2.5;
  tol[1] = 3.5;
  ros::Duration max_step(2,0);

  calibration_msgs::Interval interval = IntervalCalc::computeLatestInterval(signal, tol, max_step);
  EXPECT_EQ(interval.start.sec, (unsigned int) 6);
  EXPECT_EQ(interval.end.sec,   (unsigned int) 8);
}

// Another simple test, but this time the 2nd channel will exceed the tolerance first
TEST(IntervalCalc, easy2)
{
  SortedDeque<DeflatedConstPtr> signal = generateSignal1();

  vector<double> tol(2);
  tol[0] = 4.5;
  tol[1] = 3.5;
  ros::Duration max_step(2,0);

  calibration_msgs::Interval interval = IntervalCalc::computeLatestInterval(signal, tol, max_step);
  EXPECT_EQ(interval.start.sec, (unsigned int) 5);
  EXPECT_EQ(interval.end.sec,   (unsigned int) 8);
}

// See what happens if our max step is really small
TEST(IntervalCalc, maxStep1)
{
  SortedDeque<DeflatedConstPtr> signal = generateSignal1();

  vector<double> tol(2);
  tol[0] = 4.5;
  tol[1] = 3.5;
  ros::Duration max_step;
  max_step.fromSec(.5);

  calibration_msgs::Interval interval = IntervalCalc::computeLatestInterval(signal, tol, max_step);
  EXPECT_EQ(interval.start.sec, (unsigned int) 8);
  EXPECT_EQ(interval.end.sec,   (unsigned int) 8);
}

// See what happens if there's a big gap in time in our signal
TEST(IntervalCalc, maxStep2)
{
  SortedDeque<DeflatedConstPtr> signal = generateSignal2();

  vector<double> tol(2);
  tol[0] = 100;
  tol[1] = 100;
  ros::Duration max_step(5,0);

  calibration_msgs::Interval interval = IntervalCalc::computeLatestInterval(signal, tol, max_step);
  EXPECT_EQ(interval.start.sec, (unsigned int) 15);
  EXPECT_EQ(interval.end.sec,   (unsigned int) 18);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
