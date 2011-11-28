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
import unittest
import rospy

from capture_executive.robot_measurement_cache import RobotMeasurementCache
from calibration_msgs.msg import *

## A sample python unit test
class TestCache(unittest.TestCase):
    ## test 1 == 1
    def test_one_sensor_easy(self):
        cache = RobotMeasurementCache()
        cache.reconfigure(["cam1"], [], [])
        cache.set_max_sizes( {"cam1":100}, {}, {})

        for n in range(1,11):
            msg = CameraMeasurement()
            msg.header.stamp = rospy.Time(n*10,0)
            cache.add_cam_measurement("cam1", msg)

        m_robot = cache.request_robot_measurement(rospy.Time(19,0), rospy.Time(31,0))

        self.assertTrue(m_robot is not None)
        self.assertEquals( len(m_robot.M_cam), 1 )
        self.assertEquals( len(m_robot.M_chain), 0 )
        self.assertEquals( len(m_robot.M_laser), 0 )

        m_robot = cache.request_robot_measurement(rospy.Time(21,0), rospy.Time(29,0))
        self.assertTrue(m_robot is None)

    def test_multi_cam_easy(self):
        cache = RobotMeasurementCache()
        cache.reconfigure(["cam1", "cam2"], [], [])
        cache.set_max_sizes( {"cam1":100, "cam2":100}, {}, {})

        for n in range(1,11):
            msg = CameraMeasurement()
            msg.header.stamp = rospy.Time(n*10,0)
            cache.add_cam_measurement("cam1", msg)

        for n in range(1,11):
            msg = CameraMeasurement()
            msg.header.stamp = rospy.Time(n*10+1,0)
            cache.add_cam_measurement("cam2", msg)

        m_robot = cache.request_robot_measurement(rospy.Time(18,0), rospy.Time(32,0))

        self.assertTrue(m_robot is not None)
        self.assertEquals( len(m_robot.M_cam), 2 )
        self.assertEquals( len(m_robot.M_chain), 0 )
        self.assertEquals( len(m_robot.M_laser), 0 )

        m_robot = cache.request_robot_measurement(rospy.Time(20,0), rospy.Time(30,0))
        self.assertTrue(m_robot is None)

    def test_chain_easy(self):
        cache = RobotMeasurementCache()
        cache.reconfigure([], ["chain1"], [])
        cache.set_max_sizes( {}, {"chain1":100}, {})

        for n in range(1,11):
            msg = ChainMeasurement()
            msg.header.stamp = rospy.Time(n*10,0)
            cache.add_chain_measurement("chain1", msg)

        m_robot = cache.request_robot_measurement(rospy.Time(18,0), rospy.Time(32,0))

        self.assertTrue(m_robot is not None)
        self.assertEquals( len(m_robot.M_cam), 0 )
        self.assertEquals( len(m_robot.M_chain), 1 )
        self.assertEquals( len(m_robot.M_laser), 0 )

        m_robot = cache.request_robot_measurement(rospy.Time(21,0), rospy.Time(29,0))
        self.assertTrue(m_robot is None)

    def test_laser_easy(self):
        cache = RobotMeasurementCache()
        cache.reconfigure([], [], ["laser1"])
        cache.set_max_sizes( {}, {}, {"laser1":100})

        for n in range(1,11):
            msg = LaserMeasurement()
            cache.add_laser_measurement("laser1", msg, rospy.Time(n*10,0), rospy.Time(n*10+2,0))

        m_robot = cache.request_robot_measurement(rospy.Time(19,0), rospy.Time(32,0))

        self.assertTrue(m_robot is not None)
        self.assertEquals( len(m_robot.M_cam), 0 )
        self.assertEquals( len(m_robot.M_chain), 0 )
        self.assertEquals( len(m_robot.M_laser), 1 )

        m_robot = cache.request_robot_measurement(rospy.Time(20,0), rospy.Time(21,0))
        self.assertTrue(m_robot is None)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2_calibration_executive', 'test_robot_measurement_cache', TestCache)
