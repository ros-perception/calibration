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

//#include <stdio.h>
//#include <iostream>
#include <vector>
#include <deque>
//#include <fstream>

#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include "ros/console.h"
#include "interval_intersection/interval_intersection.hpp"

using namespace std;
using namespace interval_intersection;

IntervalIntersector::IntervalIntersector(boost::function<void (const calibration_msgs::Interval&)> output_callback):
  max_queue_size(200),
  output_callback_(output_callback)
{
}

IntervalIntersector::~IntervalIntersector()
{
  // If callbacks continue during or after destruction, bad things can happen,
  // so ensure proper shutdown of the callback streams before destruction.
}

boost::function<void (const calibration_msgs::IntervalConstPtr&)> IntervalIntersector::getNewInputStream() {
  // Changes the configuration of the set of queues,
  // so we lock all mutexes.
  boost::mutex::scoped_lock processing_lock(processing_mutex);
  size_t n = queues.size();  // Protected by above lock
  for (size_t i=0; i<n; i++) {
    mutexes[i]->lock();
  }
  queues.resize(n+1);
  queue_stats.resize(n+1);
  queue_stats[n].reset(new queue_stat());
  mutexes.resize(n+1);
  mutexes[n].reset(new boost::mutex());
  for (size_t i=0; i<n; i++) {
    mutexes[i]->unlock();
  }
  return boost::bind(&IntervalIntersector::inputCallback, this, _1, n);
}

/*!
 * \brief Interval message callback.
 *
 * It puts the message on a queue. If all queues are non-empty, they are processed
 * until one of them is empty.
 */
void IntervalIntersector::inputCallback(const calibration_msgs::IntervalConstPtr& interval_ptr, size_t i) {
  ROS_DEBUG("Got message on stream [%zu]", i);
  boost::mutex::scoped_lock lock(*mutexes[i]);
  if (queues[i].size() < max_queue_size) {
    queues[i].push_back(interval_ptr);
    queue_stats[i]->count++;
    if( interval_ptr->start == interval_ptr->end ) {
      queue_stats[i]->nil_count++;
    }
  }
  lock.unlock();

  process_queues();
}

void IntervalIntersector::process_queues() {
  //cout << "processing" << endl;
  while (1) {
    // 1) Determine which interval to publish next
    ros::Time start = ros::TIME_MIN;
    ros::Time end = ros::TIME_MAX;
    int queue_to_pop = -1;
    boost::mutex::scoped_lock processing_lock(processing_mutex);
    for (size_t i=0; i<queues.size(); i++) {
      // It looks like we could get away without locking the mutexes to
      // determine the min and max since even though the queues may change
      // because new elements are added, the first element if it exists
      // does not change because we hold the processing mutex.
      // However there is a possibility that the location of queue[i][0]
      // changes after its address has been computed but before its value
      // has been read. In other words the statement
      // "start = queues[i][0]->start" is not atomic.
      // In fact even queues[i].empty() is only guaranteed to make sense
      // in between other method calls, not during.
      // Whether or not deque is in fact thread-safe here is implementation
      // dependent.
      boost::mutex::scoped_lock lock(*mutexes[i]);
      if (queues[i].empty()) {
        // We can't determine the next interval: nothing to do
        //cout << "nothing to do" << endl;
        return;
      }
      if (queues[i][0]->start > start) {
        start = queues[i][0]->start;
      }
      if (queues[i][0]->end < end) {
        end = queues[i][0]->end;
        queue_to_pop = i;
      }
      // mutexes[i] released
    }
    if (queue_to_pop < 0) {
      ROS_ERROR("IntervalIntersection logic error");
      exit(-1);
    }
    // 2) Publish the interval
    //cout << "about to publish" << endl;
    if (start < end) {
      // Output the interval
      calibration_msgs::Interval interval;
      interval.start = start;
      interval.end = end;
      output_callback_(interval);
    }
    else
    {
      ROS_DEBUG("Publishing null interval");
      calibration_msgs::Interval interval;
      interval.start = start;
      interval.end = start;
      output_callback_(interval);
    }
    // 3) Pop the input interval with earliest end time
    boost::mutex::scoped_lock lock(*mutexes[queue_to_pop]);
    queues[queue_to_pop].pop_front();
    // mutexes[queue_to_pop] unlocks
    // processing_mutex unlocks
  }
}

calibration_msgs::IntervalStatus IntervalIntersector::get_status() {
  calibration_msgs::IntervalStatus status;
  // we only provide the raw status; the upper levels have to fill in the
  // header stamp and the names

  boost::mutex::scoped_lock processing_lock(processing_mutex);
  size_t n = queue_stats.size();
  status.names.resize(n);
  status.yeild_rates.resize(n);
  for (size_t i=0; i<n; i++) {
    boost::mutex::scoped_lock lock(*mutexes[i]);
    int count = queue_stats[i]->count;
    if( 0 == count ) {
       status.yeild_rates[i] = 0.0;
    } else {
       double good = count - queue_stats[i]->nil_count;
       status.yeild_rates[i] = good / count;
    }
  }
  return status;
}

