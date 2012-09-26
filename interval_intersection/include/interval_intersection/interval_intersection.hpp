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
#include <deque>

#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include "calibration_msgs/Interval.h"
#include "calibration_msgs/IntervalStatus.h"

using namespace std;

namespace interval_intersection
{
/** IntervalIntersector
 *
 * This class receives streams of interval messages, sorted by end time within
 * each stream. For each end time, it outputs the largest intervals contained
 * in one interval of each stream. This class is general, but is designed to
 * compute the intervals of time where a list of sensors were all stable at
 * the same time, and can therefore be used together for calibration. The code
 * assumes that start times of intervals are also non-decreasing. If not, it
 * will output a reasonable answer though not an optimal one (it is impossible
 * in that case to guarantee optimality since a later message can extend a
 * previously output interval).
 */
class IntervalIntersector {
public:
  IntervalIntersector(boost::function<void (const calibration_msgs::Interval&)> output_callback);
  ~IntervalIntersector();

  boost::function<void (const calibration_msgs::IntervalConstPtr&)> getNewInputStream();

  calibration_msgs::IntervalStatus get_status();

private:
  /*!
   * \brief Interval message callback.
   *
   * It puts the message on a queue. If all queues are non-empty, they are processed
   * until one of them is empty.
   */
  void inputCallback(const calibration_msgs::IntervalConstPtr& interval_ptr, size_t i);
  void process_queues();

  struct queue_stat {
     int count;
     int nil_count;
  };

  vector<deque<calibration_msgs::IntervalConstPtr> > queues;
  vector<boost::shared_ptr<queue_stat> > queue_stats;
  vector<boost::shared_ptr<boost::mutex> > mutexes;  // Protects the individual queues
  boost::mutex processing_mutex;  // Protects the consumption of elements from the set of queues
  size_t max_queue_size;  // must be greater than 0, or all messages will be dropped
  boost::function<void (const calibration_msgs::Interval&)> output_callback_;  // Call this function on each output interval


}; // end class

}
