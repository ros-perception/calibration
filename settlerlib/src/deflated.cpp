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

#include <settlerlib/deflated.h>
#include <ros/console.h>

using namespace settlerlib;

bool Deflated::interp(const Deflated& before, const Deflated& after,
                      const ros::Time& target, std::vector<double>& result)
{
  if (before.channels_.size() != after.channels_.size())
    return false;

  if (after.header.stamp < target)
    return false;

  if (before.header.stamp > target)
    return false;

  if (before.header.stamp == after.header.stamp)
  {
    result = before.channels_;
    return true;
  }

  double before_factor = (after.header.stamp - target).toSec()  / (after.header.stamp - before.header.stamp).toSec();
  double after_factor  = (target - before.header.stamp).toSec() / (after.header.stamp - before.header.stamp).toSec();

  const unsigned int N = before.channels_.size();
  result.resize(N);
  for (unsigned int i=0; i<N; i++)
    result[i] = before_factor*before.channels_[i] + after_factor*after.channels_[i];
  return true;
}
