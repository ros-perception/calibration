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
#include <settlerlib/deflated.h>

using namespace std;
using namespace settlerlib;

static const double eps = 1e-6;

TEST(Deflated, easy1)
{
  Deflated before, after;
  before.header.stamp = ros::Time(0,0);
  after.header.stamp = ros::Time(10,0);

  before.channels_.resize(2);
  after.channels_.resize(2);

  before.channels_[0] = 0;
  after.channels_[0]  = 100;

  before.channels_[1] = 1000;
  after.channels_[1]  = 0;

  vector<double> middle;
  bool success;
  success = Deflated::interp(before, after, ros::Time(0,0), middle);
  EXPECT_TRUE(success);
  EXPECT_NEAR(middle[0], 0, eps);
  EXPECT_NEAR(middle[1], 1000, eps);

  success = Deflated::interp(before, after, ros::Time(1,0), middle);
  EXPECT_TRUE(success);
  EXPECT_NEAR(middle[0], 10, eps);
  EXPECT_NEAR(middle[1], 900, eps);

  success = Deflated::interp(before, after, ros::Time(10,0), middle);
  EXPECT_TRUE(success);
  EXPECT_NEAR(middle[0], 100, eps);
  EXPECT_NEAR(middle[1], 0, eps);

  EXPECT_TRUE(true);
}

TEST(Deflated, zeroInterval)
{
  Deflated before, after;
  before.header.stamp = ros::Time(10,0);
  after.header.stamp  = ros::Time(10,0);
  before.channels_.resize(2);
  after.channels_.resize(2);

  before.channels_[0] = 10;
  after.channels_[0]  = 10;

  before.channels_[1] = 20;
  after.channels_[1]  = 20;

  vector<double> middle;
  bool success;
  success = Deflated::interp(before, after, ros::Time(10,0), middle);
  EXPECT_TRUE(success);

  EXPECT_NEAR(middle[0], 10, eps);
  EXPECT_NEAR(middle[1], 20, eps);
}

TEST(Deflated, sizeMismatch)
{
  Deflated before, after;
  before.header.stamp = ros::Time(0,0);
  after.header.stamp = ros::Time(10,0);
  before.channels_.resize(2);
  after.channels_.resize(3);

  vector<double> middle;
  bool success;
  success = Deflated::interp(before, after, ros::Time(0,0), middle);
  EXPECT_FALSE(success);
}

TEST(Deflated, badIntervals)
{
  Deflated before, after;
  before.channels_.resize(3);
  after.channels_.resize(3);

  vector<double> middle;

  before.header.stamp = ros::Time(10,0);
  after.header.stamp =  ros::Time(20,0);
  EXPECT_FALSE(Deflated::interp(before, after, ros::Time( 5,0), middle));
  EXPECT_TRUE( Deflated::interp(before, after, ros::Time(15,0), middle));
  EXPECT_FALSE(Deflated::interp(before, after, ros::Time(25,0), middle));

  before.header.stamp = ros::Time(20,0);
  after.header.stamp  = ros::Time(10,0);
  EXPECT_FALSE(Deflated::interp(before, after, ros::Time( 5,0), middle));
  EXPECT_FALSE(Deflated::interp(before, after, ros::Time(15,0), middle));
  EXPECT_FALSE(Deflated::interp(before, after, ros::Time(25,0), middle));
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
