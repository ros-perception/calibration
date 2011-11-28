/*********************************************************************
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

//! \author Vijay Pradeep

#include <sstream>
#include <joint_states_settler/joint_states_settler.h>
#include <settlerlib/interval_calc.h>
#include <ros/console.h>

using namespace joint_states_settler;

JointStatesSettler::JointStatesSettler() : cache_(&DeflatedMsgCache::getPtrStamp)
{
  configured_ = false;
}

bool JointStatesSettler::configure(const joint_states_settler::ConfigGoal& goal)
{
  const unsigned int N = goal.joint_names.size();

  if (N != goal.tolerances.size())
  {
    ROS_ERROR("Invalid configuration for JointStatesSettler:\n"
              "  [joint_names.size() == %u] should equal [tolerances.size() == %u]",
              goal.joint_names.size(), goal.tolerances.size());
    return false;
  }

  deflater_.setDeflationJointNames(goal.joint_names);
  tol_ = goal.tolerances;
  max_step_ = goal.max_step;
  cache_.clear();
  cache_.setMaxSize(goal.cache_size);

  std::ostringstream info;
  info << "Configuring JointStatesSettler with the following joints:";
  for (unsigned int i=0; i<N; i++)
    info << "\n   " << goal.joint_names[i] << ": " << goal.tolerances[i];
  ROS_DEBUG(info.str().c_str());

  configured_ = true;
  return true;
}

calibration_msgs::Interval JointStatesSettler::add(const sensor_msgs::JointStateConstPtr msg)
{
  if (!configured_)
  {
    ROS_WARN("Not yet configured. Going to skip");
    return calibration_msgs::Interval();
  }

  boost::shared_ptr<DeflatedJointStates> deflated(new DeflatedJointStates);
  deflater_.deflate(msg, *deflated);
  cache_.add(deflated);

  DeflatedCache* bare_cache = (DeflatedCache*)(&cache_);

  calibration_msgs::Interval interval = settlerlib::IntervalCalc::computeLatestInterval(*bare_cache, tol_, max_step_);

  return interval;
}

sensor_msgs::JointState JointStatesSettler::pruneJointState(const sensor_msgs::JointStateConstPtr msg)
{
  sensor_msgs::JointState pruned;
  deflater_.prune(*msg, pruned);
  return pruned;
}
