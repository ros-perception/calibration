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

//! \author Vijay Pradeep

#include <sstream>
#include <settlerlib/interval_calc.h>
#include <ros/console.h>

#define INTERVAL_DEBUG(fmt, ...) \
    ROS_DEBUG_NAMED("IntervalCalc", fmt,##__VA_ARGS__)


using namespace std;
using namespace settlerlib;

calibration_msgs::Interval IntervalCalc::computeLatestInterval(const SortedDeque<DeflatedConstPtr>& signal,
                                                               const std::vector<double>& tolerances,
                                                               ros::Duration max_spacing)
{
  if (max_spacing < ros::Duration(0,0))
  {
    ROS_WARN("max_spacing is negative (%.3f). Should be positive", max_spacing.toSec());
    max_spacing = -max_spacing;
  }

  if (signal.size() == 0)
  {
    ROS_WARN("Can't compute range of an empty signal");
    return calibration_msgs::Interval();
  }

  std::deque<DeflatedConstPtr>::const_reverse_iterator rev_it = signal.rbegin();

  assert(*rev_it);  // Make sure it's not a NULL pointer

  const unsigned int N = (*rev_it)->channels_.size();

  assert(tolerances.size() == N);
  vector<double> channel_max( (*rev_it)->channels_ );
  vector<double> channel_min( (*rev_it)->channels_ );
  vector<double> channel_range(N);

  calibration_msgs::Interval result;
  result.end   = (*signal.rbegin())->header.stamp;
  result.start = result.end;


  INTERVAL_DEBUG("Starting to walk along interval:");
  while( rev_it != signal.rend() )
  {
    if ( (*rev_it)->channels_.size() != N)
    {
      ROS_WARN("Num channels has changed. Cutting off interval prematurely ");
      return result;
    }

    ros::Duration cur_step = result.start - (*rev_it)->header.stamp;
    if ( cur_step > max_spacing)
    {
      INTERVAL_DEBUG("Difference between interval.start and it.stamp is [%.3fs]"
                     "Exceeds [%.3fs]", cur_step.toSec(), max_spacing.toSec());
      return result;
    }

    ostringstream max_debug;
    ostringstream min_debug;
    ostringstream range_debug;
    max_debug   << "  max:  ";
    min_debug   << "  min:  ";
    range_debug << "  range:";
    for (unsigned int i=0; i<N; i++)
    {
      channel_max[i]   = fmax( channel_max[i], (*rev_it)->channels_[i] );
      channel_min[i]   = fmin( channel_min[i], (*rev_it)->channels_[i] );
      channel_range[i] = channel_max[i] - channel_min[i];

      max_debug << "  " << channel_max[i];
      min_debug << "  " << channel_min[i];
      range_debug << "  " << channel_range[i];
    }

    INTERVAL_DEBUG("Current stats:\n%s\n%s\n%s",
                   max_debug.str().c_str(),
                   min_debug.str().c_str(),
                   range_debug.str().c_str());

    for (unsigned int i=0; i<N; i++)
    {
      if (channel_range[i] > tolerances[i])
      {
        INTERVAL_DEBUG("Channel %u range is %.3f.  Exceeds tolerance of %.3f", i, channel_range[i], tolerances[i]);
        return result;
      }
    }

    result.start = (*rev_it)->header.stamp;

    rev_it++;
  }
  return result;
}
