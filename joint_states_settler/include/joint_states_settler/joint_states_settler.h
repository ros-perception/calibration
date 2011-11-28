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

#ifndef JOINT_STATES_SETTLER_JOINT_STATES_SETTLER_H_
#define JOINT_STATES_SETTLER_JOINT_STATES_SETTLER_H_

#include <boost/shared_ptr.hpp>

#include <calibration_msgs/Interval.h>
#include <sensor_msgs/JointState.h>

#include <settlerlib/sorted_deque.h>

#include <joint_states_settler/ConfigGoal.h>

#include "joint_states_deflater.h"
#include "deflated_joint_states.h"

namespace joint_states_settler
{

class JointStatesSettler
{
public:
  JointStatesSettler();

  bool configure(const joint_states_settler::ConfigGoal& goal);

  calibration_msgs::Interval add(const sensor_msgs::JointStateConstPtr msg);
  sensor_msgs::JointState pruneJointState(const sensor_msgs::JointStateConstPtr msg);

private:
  bool configured_;
  JointStatesDeflater deflater_;
  std::vector<double> tol_;
  ros::Duration max_step_;

  typedef settlerlib::SortedDeque< boost::shared_ptr<const DeflatedJointStates> > DeflatedMsgCache;
  typedef settlerlib::SortedDeque<settlerlib::DeflatedConstPtr> DeflatedCache;
  DeflatedMsgCache cache_;

};

}

#endif
