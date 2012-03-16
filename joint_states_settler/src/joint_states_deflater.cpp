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

#include "joint_states_settler/joint_states_deflater.h"
#include <ros/console.h>

using namespace std;
using namespace joint_states_settler;

JointStatesDeflater::JointStatesDeflater()
{
  mapping_.clear();
}

void JointStatesDeflater::setDeflationJointNames(std::vector<std::string> joint_names)
{
  joint_names_ = joint_names;
  mapping_.resize(joint_names_.size());
}

void JointStatesDeflater::deflate(const sensor_msgs::JointStateConstPtr& joint_states, DeflatedJointStates& deflated_elem)
{
  if (joint_states->name.size() != joint_states->position.size()){
    ROS_ERROR("JointStatesDeflater got invalid joint state message");
    return;
  }

  if (mapping_.size() != joint_names_.size())
    updateMapping(*joint_states);

  const unsigned int N = joint_names_.size();

  deflated_elem.channels_.resize(N);

  for (unsigned int i=0; i<N; i++)
  {
    if ( mapping_[i] >= joint_states->name.size() )
      updateMapping(*joint_states);

    if ( joint_states->name[mapping_[i]] != joint_names_[i])
      updateMapping(*joint_states);

    deflated_elem.header = joint_states->header;
    deflated_elem.channels_[i] = joint_states->position[mapping_[i]];
    deflated_elem.msg_ = joint_states;
  }
}

void JointStatesDeflater::prune(const sensor_msgs::JointState& joint_states, sensor_msgs::JointState& pruned_joint_states)
{
  if (joint_states.name.size() !=  joint_states.position.size())
  {
    ROS_ERROR("JointStatesDeflater got invalid joint state message. name's size and position's size don't match.");
    return;
  }

  if (mapping_.size() != joint_names_.size())
    updateMapping(joint_states);

  const unsigned int N = joint_names_.size();

  // Initialize Structures
  pruned_joint_states.header.stamp = joint_states.header.stamp;
  pruned_joint_states.name.resize(N);
  pruned_joint_states.position.resize(N);
  pruned_joint_states.velocity.clear();
  pruned_joint_states.effort.clear();

  for (unsigned int i=0; i<N; i++)
  {
    if ( mapping_[i] >= joint_states.name.size() )
      updateMapping(joint_states);

    if ( joint_states.name[mapping_[i]] != joint_names_[i])
      updateMapping(joint_states);

    pruned_joint_states.name[i]     = joint_states.name[mapping_[i]];
    pruned_joint_states.position[i] = joint_states.position[mapping_[i]];

    // Currently not copying over velocity or effort. Maybe we should be doing this...
  }
}

void JointStatesDeflater::updateMapping(const sensor_msgs::JointState& joint_states)
{
  ROS_DEBUG("Updating the JointStates mapping");

  const unsigned int N = joint_names_.size();

  mapping_.resize(N);

  for (unsigned int i=0; i<N; i++)
  {
    bool mapping_found = false;
    for (unsigned int j=0; j<joint_states.name.size(); j++)
    {
      if ( joint_names_[i] == joint_states.name[j])
      {
        mapping_[i] = j;
        mapping_found = true;
      }
    }
    ROS_ERROR_COND(!mapping_found, "Couldn't find mapping for [%s]", joint_names_[i].c_str());
  }
}
