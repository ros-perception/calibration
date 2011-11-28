/**********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

// Tests interval_intersector.cpp, but it does not test with concurrency.


#include <vector>
#include <string>
#include <sstream>
#include <gtest/gtest.h>

#include "calibration_msgs/Interval.h"
#include "interval_intersection/interval_intersection.hpp"

using namespace std;
using namespace interval_intersection;

stringstream output_stream;
typedef boost::function<void (const calibration_msgs::IntervalConstPtr &)> Input;
vector<Input> inputs;

void publish(const calibration_msgs::Interval &interval)
{
  //cout << "publishing" << endl;
  output_stream << "[" << interval.start << ", " << interval.end << "] ";
}

void input_interval(ros::Time start, ros::Time end, int input_num) {
  boost::shared_ptr<calibration_msgs::Interval> interval(new calibration_msgs::Interval());
  interval->start = start;
  interval->end = end;
  //cout << "inputing" << endl;
  inputs[input_num](interval);
}

TEST(Intersection, testCase1)
{
  // Creat IntervalIntersector and hook its inputs and output
  IntervalIntersector intersector(&publish);
  int n = 3;
  for (int i=0; i<n; i++) {
    inputs.push_back(intersector.getNewInputStream());
  }

  // Feed it some input
  typedef ros::Time T;
  input_interval(T(10), T(20), 0);
  input_interval(T(2), T(16), 1);
  input_interval(T(8), T(21), 2);
  input_interval(T(23), T(30), 0);
  input_interval(T(6), T(25), 1);
  input_interval(T(18), T(28), 2);

  string correct_answer = "[10.000000000, 16.000000000] [10.000000000, 20.000000000] [23.000000000, 23.000000000] [23.000000000, 25.000000000] ";
  EXPECT_EQ(output_stream.str(), correct_answer);
}


// Main
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


