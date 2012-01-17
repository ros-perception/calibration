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

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <joint_states_settler/joint_states_settler.h>
#include <joint_states_settler/ConfigAction.h>
#include <calibration_msgs/Interval.h>
#include <sensor_msgs/JointState.h>

using namespace joint_states_settler;

class JointStatesSettlerAction
{
public:
  JointStatesSettlerAction() : as_("settler_config", false)
  {
    as_.registerGoalCallback( boost::bind(&JointStatesSettlerAction::goalCallback, this) );
    as_.registerPreemptCallback( boost::bind(&JointStatesSettlerAction::preemptCallback, this) );
    interval_pub_ = nh_.advertise<calibration_msgs::Interval>("settled_interval", 1);
    pruned_pub_ = nh_.advertise<sensor_msgs::JointState>("chain_state", 1);
    sub_ = nh_.subscribe("joint_states", 1, &JointStatesSettlerAction::jointStatesCallback, this);
    as_.start();
  }

  void goalCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    // Stop the previously running goal (if it exists)
    if (as_.isActive())
      as_.setPreempted();

    // Get the new goal from the action server
    joint_states_settler::ConfigGoalConstPtr goal = as_.acceptNewGoal();
    assert(goal);

    // Try to reconfigure the settler object
    bool success = settler_.configure(*goal);

    // Detect possible failure
    if (!success)
      as_.setAborted();
  }

  void preemptCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    // Don't need to do any cleanup. Immeadiately turn it off
    as_.setPreempted();
  }

  void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    // Don't do anything if we're not actually running
    if (!as_.isActive())
      return;

    // Add the joint state message, and get the latest processed settled interval
    calibration_msgs::Interval interval = settler_.add(msg);
    interval_pub_.publish(interval);

    // Build the joint state message for this subset of joints
    pruned_pub_.publish(settler_.pruneJointState(msg));
  }

private:
  boost::mutex run_mutex_;
  actionlib::SimpleActionServer<joint_states_settler::ConfigAction> as_;
  JointStatesSettler settler_;

  ros::Publisher interval_pub_;
  ros::Publisher pruned_pub_;
  ros::Subscriber sub_;
  ros::NodeHandle nh_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_settler_action");
  ros::NodeHandle n;
  JointStatesSettlerAction settler_action;
  ros::spin();
  return 0;
}
