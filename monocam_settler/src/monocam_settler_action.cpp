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
#include <monocam_settler/monocam_settler.h>

// Messages
#include <monocam_settler/ConfigAction.h>
#include <calibration_msgs/CalibrationPattern.h>
#include <calibration_msgs/Interval.h>

using namespace monocam_settler;

class MonocamSettlerAction
{
public:
  MonocamSettlerAction() : as_("monocam_settler_config", false)
  {
    // Set up action callbacks
    as_.registerGoalCallback( boost::bind(&MonocamSettlerAction::goalCallback, this) );
    as_.registerPreemptCallback( boost::bind(&MonocamSettlerAction::preemptCallback, this) );

    pub_ = nh_.advertise<calibration_msgs::Interval>("settled_interval", 1);
    sub_ = nh_.subscribe("features", 1, &MonocamSettlerAction::msgCallback, this);
    as_.start();
  }

  void goalCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    // Stop the previously running goal (if it exists)
    if (as_.isActive())
      as_.setPreempted();

    // Get the new goal from the action server
    monocam_settler::ConfigGoalConstPtr goal = as_.acceptNewGoal();
    assert(goal);

    // Try to reconfigure the settler object
    bool success = settler_.configure(*goal);

    // Detect possible reconfigure failure
    if (!success)
      as_.setAborted();
  }

  void preemptCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);
    as_.setPreempted();
  }

  void msgCallback(const calibration_msgs::CalibrationPatternConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    if (as_.isActive())
    {
      bool success;

      calibration_msgs::Interval interval;
      success = settler_.add(msg, interval);

      if (success)
        pub_.publish(interval);
      else
      {
        // Publish 'null' interval if we didn't find a checkerboard
        interval.start = msg->header.stamp;
        interval.end =   msg->header.stamp;
        pub_.publish(interval);
      }
    }
    else
      ROS_DEBUG("Got a feature msg, but not doing anything with it. No active goal, so node is currently idle");
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<monocam_settler::ConfigAction> as_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  boost::mutex run_mutex_;
  MonocamSettler settler_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_settler_action");
  ros::NodeHandle n;
  MonocamSettlerAction settler_action;
  ros::spin();
  return 0;
}

