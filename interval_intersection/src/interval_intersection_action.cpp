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

#include <vector>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <interval_intersection/interval_intersection.hpp>

#include <calibration_msgs/Interval.h>
#include <calibration_msgs/IntervalStatus.h>
#include <interval_intersection/ConfigAction.h>

using namespace std;
using namespace interval_intersection;

class IntervalIntersectionAction
{
public:
  IntervalIntersectionAction() : as_("interval_intersection_config", false)
  {
    as_.registerGoalCallback(    boost::bind(&IntervalIntersectionAction::goalCallback, this) );
    as_.registerPreemptCallback( boost::bind(&IntervalIntersectionAction::preemptCallback, this) );
    pub_ = nh_.advertise<calibration_msgs::Interval>("intersected_interval", 1);
    status_pub_ = nh_.advertise<calibration_msgs::IntervalStatus>(
          "intersected_interval_status", 1);

    // set up our status timer
    status_timer_ = nh_.createTimer(ros::Duration(1.0),
        boost::bind(&IntervalIntersectionAction::publishStatus, this));

    ROS_DEBUG("Start interval intersection with no input topics");
    as_.start();
  }

  void goalCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    // Stop currently running goal, if there's a goal running
    if (as_.isActive())
    {
      tearDown();
      as_.setPreempted();
    }

    // Get the new goal from the action server
    interval_intersection::ConfigGoalConstPtr goal = as_.acceptNewGoal();
    assert(goal);

    // Reconfigure the node
    intersect_nh_.reset(new ros::NodeHandle);
    intersect_.reset(new IntervalIntersector( boost::bind(&IntervalIntersectionAction::publishResult, this, _1)));
    // Subscribe to all the requested topics
    subscribers_.resize(goal->topics.size());
    topics_.resize(goal->topics.size());
    for (unsigned int i=0; i<goal->topics.size(); i++)
    {
      ROS_DEBUG("Subscribing to: %s", goal->topics[i].c_str());
      subscribers_[i] = intersect_nh_->subscribe<calibration_msgs::Interval>(goal->topics[i], 200, intersect_->getNewInputStream());
      topics_[i] = goal->topics[i];
    }
  }

  void preemptCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);
    tearDown();
    as_.setPreempted();
  }

  void tearDown()
  {
    assert(intersect_);
    assert(intersect_nh_);

    // Shutdown the subnode, to ensure that none of the callbacks are called
    ROS_DEBUG("Shutting Down IntervalIntersection");
    intersect_nh_->shutdown();

    subscribers_.clear();
    intersect_.reset();
    intersect_nh_.reset();
  }

  void publishResult(calibration_msgs::Interval interval)
  {
    ROS_DEBUG("Publishing");
    pub_.publish(interval);
  }

  void publishStatus() {
     ROS_DEBUG("Publishing interval intersection status");
     if ( intersect_ ) {
        // get status from intersector
        calibration_msgs::IntervalStatus status = intersect_->get_status();
        // fill in timestamp
        status.header.stamp = ros::Time::now();
        // fill in topic names
        for ( size_t i=0; i<status.yeild_rates.size(); ++i ) {
           status.names[i] = topics_[i];
           ROS_DEBUG("Topic %s has a success rate of %f", 
                 status.names[i].c_str(), status.yeild_rates[i]);
        }
        // publish!
        status_pub_.publish(status);
     }
  }

private:
  boost::mutex run_mutex_;
  actionlib::SimpleActionServer<interval_intersection::ConfigAction> as_;

  boost::scoped_ptr<ros::NodeHandle> intersect_nh_;
  boost::scoped_ptr<IntervalIntersector> intersect_;
  vector<ros::Subscriber> subscribers_;
  vector<std::string> topics_;

  ros::Publisher pub_;
  ros::Publisher status_pub_;
  ros::NodeHandle nh_;
  ros::Timer status_timer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intersection_action_node");
  IntervalIntersectionAction intersect_action;
  ros::spin();
  return 0;
}
