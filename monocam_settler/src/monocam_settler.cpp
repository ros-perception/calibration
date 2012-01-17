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

#include <monocam_settler/monocam_settler.h>
#include <settlerlib/interval_calc.h>

using namespace monocam_settler;


MonocamSettler::MonocamSettler() : cache_(&DeflatedMsgCache::getPtrStamp)
{
  configured_ = false;
}

bool MonocamSettler::configure(const monocam_settler::ConfigGoal& goal)
{
  tol_ = goal.tolerance;
  max_step_ = goal.max_step;
  ignore_failures_ = goal.ignore_failures;
  cache_.clear();
  cache_.setMaxSize(goal.cache_size);

  ROS_DEBUG("Configuring MonocamSettler with tolerance of [%.3f]", tol_);

  configured_ = true;
  return true;
}


bool MonocamSettler::add(const calibration_msgs::CalibrationPatternConstPtr msg,
                         calibration_msgs::Interval& interval)
{
  if (!configured_)
  {
    ROS_WARN("Not configured. Going to skip");
    return false;
  }

  // Check if detection failed
  if (!msg->success)
  {
    if(!ignore_failures_)   // If we care about failures then we should reset the cache
      cache_.clear();
    return false;
  }

  boost::shared_ptr<DeflatedCalibrationPattern> deflated(new DeflatedCalibrationPattern);
  deflate(msg, *deflated);
  cache_.add(deflated);

  DeflatedCache* bare_cache = (DeflatedCache*)(&cache_);

  std::vector<double> tol_vec(deflated->channels_.size(), tol_);

  interval = settlerlib::IntervalCalc::computeLatestInterval(*bare_cache, tol_vec, max_step_);

  return true;
}

void MonocamSettler::deflate(const calibration_msgs::CalibrationPatternConstPtr& msg,
                             DeflatedCalibrationPattern& deflated)
{
  deflated.header.stamp = msg->header.stamp;

  const unsigned int N = msg->image_points.size();
  deflated.channels_.resize( 2 * N );
  for (unsigned int i=0; i<N; i++)
  {
    deflated.channels_[2*i+0] = msg->image_points[i].x;
    deflated.channels_[2*i+1] = msg->image_points[i].y; 
  }
  deflated.msg_ = msg;
}
