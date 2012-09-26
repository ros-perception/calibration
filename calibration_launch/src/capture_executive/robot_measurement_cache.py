#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('calibration_launch')
import sys

import rospy
from calibration_msgs.msg import *
from bisect import bisect

def stamped_cmp(x,y):
    return cmp(x.header.stamp, y.header.stamp)

def laser_cmp(x,y):
    return cmp(x[0].header.stamp, y[0].header.stamp)

# Store time histories of many camera, chain, and laser measurements. These stored info
# then used when a robot_measurement is requested.
class RobotMeasurementCache:
    def __init__(self):
        self._cam_sizes   = dict()
        self._laser_sizes = dict()
        self._chain_sizes = dict()
        pass

    # Specify what sensor data we're expecting to receive
    def reconfigure(self, cam_ids, chain_ids, laser_ids):
        self._cam_caches   = dict( [ (x,[]) for x in cam_ids  ] )
        self._laser_caches = dict( [ (x,[]) for x in laser_ids] )
        self._chain_caches = dict( [ (x,[]) for x in chain_ids] )

        # Specify default max sizes, if they don't exist yet
        for cam_id in [x for x in cam_ids if x not in self._cam_sizes]:
            self._cam_sizes[cam_id] = 100
        for chain_id in [x for x in chain_ids if x not in self._chain_sizes]:
            self._chain_sizes[chain_id] = 3000
        for laser_id in [x for x in laser_ids if x not in self._laser_sizes]:
            self._laser_sizes[laser_id] = 5

    # Store the max sizes for all the caches as dictionaries with (id, max_size) pairs
    def set_max_sizes(self, cam_sizes, chain_sizes, laser_sizes):
        self._cam_sizes   = cam_sizes
        self._laser_sizes = laser_sizes
        self._chain_sizes = chain_sizes

    def add_cam_measurement(self, cam_id, m):
        cur_cache = self._cam_caches[cam_id]
        cur_cache.append( m )
        cur_cache.sort(stamped_cmp)
        while len(cur_cache) > self._cam_sizes[cam_id]:
            cur_cache.pop(0)

    def add_chain_measurement(self, chain_id, m):
        cur_cache = self._chain_caches[chain_id]
        cur_cache.append( m )
        cur_cache.sort(stamped_cmp)
        while len(cur_cache) > self._chain_sizes[chain_id]:
            cur_cache.pop(0)

    def add_laser_measurement(self, laser_id, m, interval_start, interval_end):
        cur_cache = self._laser_caches[laser_id]
        cur_cache.append( [m, interval_start, interval_end] )
        cur_cache.sort(laser_cmp)
        while len(cur_cache) > self._laser_sizes[laser_id]:
            cur_cache.pop(0)

    def request_robot_measurement(self, interval_start, interval_end, min_duration = rospy.Duration(2,0)):
        # Compute the center of the req interval
        req_duration = interval_end   - interval_start
        req_center   = interval_start + rospy.Duration(req_duration.to_sec()*.5)

        if req_duration < min_duration:
            return None

        # Extract the cam measurements closest to the center of the interval
        cam_measurements = dict( [ (x, None) for x in self._cam_caches.keys() ] )
        for cam_id in self._cam_caches.keys():
            # Get the index elem that is [approx] closest to the center of the interval
            cur_cache = self._cam_caches[cam_id]
            right_center_index = bisect( [x.header.stamp for x in cur_cache], req_center )

            # Make sure the index makes sense, and is in the interval
            #   Note: This is not a 'complete' or 'exact' algorithm. It can definitely be improved... if we care
            if right_center_index >= len(cur_cache):
                cam_measurements[cam_id] = None
            elif cur_cache[right_center_index].header.stamp > interval_end or cur_cache[right_center_index].header.stamp < interval_start:
                cam_measurements[cam_id] = None
            else:
                cam_measurements[cam_id] = cur_cache[right_center_index]

        # Extract the chain measurements closest to the center of the interval
        chain_measurements = dict( [ (x, None) for x in self._chain_caches.keys() ] )
        for chain_id in self._chain_caches.keys():
            # Get the index elem that is [approx] closest to the center of the interval
            cur_cache = self._chain_caches[chain_id]
            right_center_index = bisect( [x.header.stamp for x in cur_cache], req_center )

            # Make sure the index makes sense, and is in the interval
            #   Note: This is not a 'complete' or 'exact' algorithm. It can definitely be improved... if we care
            if right_center_index >= len(cur_cache):
                chain_measurements[chain_id] = None
            elif cur_cache[right_center_index].header.stamp > interval_end or cur_cache[right_center_index].header.stamp < interval_start:
                chain_measurements[chain_id] = None
            else:
                chain_measurements[chain_id] = cur_cache[right_center_index]

        # Use a slightly different strategy for the laser. Get all of them, and then choose the first one
        laser_measurements = dict( [ (x, None) for x in self._laser_caches.keys() ] )
        for laser_id in self._laser_caches.keys():
            cur_cache = self._laser_caches[laser_id]
            sub_list = [x for x in cur_cache if x[2] <= interval_end and x[1] >= interval_start]
            if len(sub_list) > 0:
                laser_measurements[laser_id] = sub_list[0][0]
            else:
                laser_measurements[laser_id] = None

        failed_sensors = []
        good_sensors = []

        # See if we got everything that we needed
        for cam_id, m in cam_measurements.items():
            if m is None:
                failed_sensors.append(cam_id)
            else:
                good_sensors.append(cam_id)

        for chain_id, m in chain_measurements.items():
            if m is None:
                failed_sensors.append(chain_id)
            else:
                good_sensors.append(chain_id)

        for laser_id, m in laser_measurements.items():
            if m is None:
                failed_sensors.append(laser_id)
            else:
                good_sensors.append(laser_id)

        if len(failed_sensors) > 0:
           if len(failed_sensors) > 1:
               m= "Failed to capture samples from %s"%(', '.join(failed_sensors))
           else:
               m= "Failed to capture a sample from %s"%(failed_sensors[0])
           return m

        # TODO: eliminate print statement or convert to rospy.log*
        print "Received everything!"

        # Push everything into a RobotMeasurement message
        m_robot = RobotMeasurement()
        m_robot.M_cam   = cam_measurements.values()
        m_robot.M_chain = chain_measurements.values()
        m_robot.M_laser = laser_measurements.values()

        return m_robot

