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
#include "joint_states_settler/joint_states_deflater.h"

using namespace std;
using namespace joint_states_settler;
using namespace sensor_msgs;

static const double eps = 1e-10;

TEST(JointStatesDeflator, easy1)
{
  JointStatesDeflater deflater;

  vector<string> joint_names;
  joint_names.resize(2);
  joint_names[0] = "A";
  joint_names[1] = "C";
  deflater.setDeflationJointNames(joint_names);

  JointStatePtr joint_states;
  DeflatedJointStates deflated_msg;
  JointState pruned;

  // ***********************************************

  joint_states.reset(new JointState);
  joint_states->name.resize(3);
  joint_states->position.resize(3);
  joint_states->velocity.resize(3);
  joint_states->effort.resize(3);
  joint_states->name[0] = "A";
  joint_states->name[1] = "B";
  joint_states->name[2] = "C";
  joint_states->position[0] = 1.0;
  joint_states->position[1] = 2.0;
  joint_states->position[2] = 3.0;

  deflater.deflate(joint_states, deflated_msg);
  deflater.prune(*joint_states, pruned);

  ASSERT_EQ(deflated_msg.channels_.size(), (unsigned int) 2);
  EXPECT_NEAR(deflated_msg.channels_[0], 1.0, eps);
  EXPECT_NEAR(deflated_msg.channels_[1], 3.0, eps);

  ASSERT_EQ(pruned.name.size(), (unsigned int) 2);
  ASSERT_EQ(pruned.position.size(), (unsigned int) 2);
  EXPECT_EQ(pruned.name[0], "A");
  EXPECT_EQ(pruned.name[1], "C");
  EXPECT_NEAR(pruned.position[0], 1.0, eps);
  EXPECT_NEAR(pruned.position[1], 3.0, eps);

  // ***********************************************

  joint_states.reset(new JointState);
  joint_states->name.resize(3);
  joint_states->position.resize(3);
  joint_states->velocity.resize(3);
  joint_states->effort.resize(3);
  joint_states->name[0] = "C";
  joint_states->name[1] = "A";
  joint_states->name[2] = "B";
  joint_states->position[0] = 4.0;
  joint_states->position[1] = 5.0;
  joint_states->position[2] = 6.0;

  deflater.deflate(joint_states, deflated_msg);
  deflater.prune(*joint_states, pruned);

  EXPECT_EQ(deflated_msg.channels_.size(), (unsigned int) 2);
  EXPECT_NEAR(deflated_msg.channels_[0], 5.0, eps);
  EXPECT_NEAR(deflated_msg.channels_[1], 4.0, eps);

  ASSERT_EQ(pruned.name.size(), (unsigned int) 2);
  ASSERT_EQ(pruned.position.size(), (unsigned int) 2);
  EXPECT_EQ(pruned.name[0], "A");
  EXPECT_EQ(pruned.name[1], "C");
  EXPECT_NEAR(pruned.position[0], 5.0, eps);
  EXPECT_NEAR(pruned.position[1], 4.0, eps);

  // ***********************************************

  joint_states.reset(new JointState);
  joint_states->name.resize(4);
  joint_states->position.resize(4);
  joint_states->velocity.resize(4);
  joint_states->effort.resize(4);
  joint_states->name[0] = "D";
  joint_states->name[1] = "C";
  joint_states->name[2] = "B";
  joint_states->name[3] = "A";
  joint_states->position[0] = 7.0;
  joint_states->position[1] = 8.0;
  joint_states->position[2] = 9.0;
  joint_states->position[3] = 10.0;

  deflater.deflate(joint_states, deflated_msg);
  deflater.prune(*joint_states, pruned);

  EXPECT_EQ(deflated_msg.channels_.size(), (unsigned int) 2);
  EXPECT_NEAR(deflated_msg.channels_[0], 10.0, eps);
  EXPECT_NEAR(deflated_msg.channels_[1], 8.0, eps);

  ASSERT_EQ(pruned.name.size(), (unsigned int) 2);
  ASSERT_EQ(pruned.position.size(), (unsigned int) 2);
  EXPECT_EQ(pruned.name[0], "A");
  EXPECT_EQ(pruned.name[1], "C");
  EXPECT_NEAR(pruned.position[0], 10.0, eps);
  EXPECT_NEAR(pruned.position[1],  8.0, eps);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
