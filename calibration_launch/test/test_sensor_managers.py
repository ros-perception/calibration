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
import time

from capture_executive.sensor_managers import CamManager
from capture_executive.sensor_managers import ChainManager
from capture_executive.sensor_managers import LaserManager
from calibration_msgs.msg import *
from sensor_msgs.msg import *

def build_msg(type, secs):
    msg = type()
    msg.header.stamp = rospy.Time(secs, 0)
    return msg

class TestChainManager(unittest.TestCase):
    def test_easy(self):
        self._cb_chain_id = None
        self._cb_msg = None
        self._cb_called = False
        manager = ChainManager("chain1", self.callback)
        manager.enable()

        chain_state_pub = rospy.Publisher("chain1/chain_state", sensor_msgs.msg.JointState)

        time.sleep(1.0)

        # Send a full set
        manager.enable()
        self._cb_called = False
        chain_state_pub.publish(build_msg(JointState, 1))
        time.sleep(1.0)
        self.assertTrue(self._cb_called)

        # disable chain manager, and then send data
        manager.disable()
        self._cb_called = False
        chain_state_pub.publish(build_msg(JointState, 2))
        time.sleep(1.0)
        self.assertFalse(self._cb_called)

    def callback(self, chain_id, msg):
        print "Calling user callback"
        self._cb_called = True
        self._cb_chain_id = chain_id
        self._cb_msg = msg

class TestCamManager(unittest.TestCase):
    def test_easy(self):
        self._cb_cam_id = None
        self._cb_msg = None
        self._cb_called = False
        manager = CamManager("cam1", self.callback)
        manager.enable(verbose=False)

        cam_info_pub = rospy.Publisher("cam1/camera_info", sensor_msgs.msg.CameraInfo)
        features_pub = rospy.Publisher("cam1/features", calibration_msgs.msg.CalibrationPattern)
        image_pub    = rospy.Publisher("cam1/image",    sensor_msgs.msg.Image)

        time.sleep(1.0)

        # Send a full set
        manager.enable(verbose=True)
        self._cb_called = False
        cam_info_pub.publish(build_msg(CameraInfo, 1))
        features_pub.publish(build_msg(CalibrationPattern, 1))
        image_pub.publish(build_msg(Image, 1))
        time.sleep(1.0)
        self.assertTrue(self._cb_called)

        # Only send a few of the messages. The verbose synchronizer shouldn't find a match
        manager.enable(verbose=True)
        self._cb_called = False
        cam_info_pub.publish(build_msg(CameraInfo, 2))
        features_pub.publish(build_msg(CalibrationPattern, 2))
        time.sleep(1.0)
        self.assertFalse(self._cb_called)

        # Only send a few of the messages again. The concise synchronizer should find a match
        manager.enable(verbose=False)
        self._cb_called = False
        cam_info_pub.publish(build_msg(CameraInfo, 3))
        features_pub.publish(build_msg(CalibrationPattern, 3))
        time.sleep(1.0)
        self.assertTrue(self._cb_called)


    def callback(self, cam_id, msg):
        print "Calling user callback"
        self._cb_called = True
        self._cb_cam_id = cam_id
        self._cb_msg = msg

class TestLaserManager(unittest.TestCase):
    def test_easy(self):
        self._cb_laser_id = None
        self._cb_msg = None
        self._cb_called = False
        manager = LaserManager("laser1", self.callback)
        manager.enable(verbose=False)

        snapshot_pub       = rospy.Publisher("laser1/snapshot",       calibration_msgs.msg.DenseLaserSnapshot)
        laser_image_pub    = rospy.Publisher("laser1/laser_image",    sensor_msgs.msg.Image)
        image_features_pub = rospy.Publisher("laser1/image_features", calibration_msgs.msg.CalibrationPattern)
        joint_features_pub = rospy.Publisher("laser1/joint_features", calibration_msgs.msg.JointStateCalibrationPattern)
        duration_pub       = rospy.Publisher("laser1/laser_interval", calibration_msgs.msg.IntervalStamped)

        time.sleep(1.0)

        # Send a full set
        manager.enable(verbose=True)
        self._cb_called = False
        snapshot_pub.publish(build_msg(DenseLaserSnapshot, 1))
        laser_image_pub.publish(build_msg(Image, 1))
        image_features_pub.publish(build_msg(CalibrationPattern, 1))
        joint_features_pub.publish(build_msg(JointStateCalibrationPattern, 1))
        duration_pub.publish(build_msg(IntervalStamped, 1))
        time.sleep(1.0)
        self.assertTrue(self._cb_called)

        # Only send a few of the messages. The verbose synchronizer shouldn't find a match
        manager.enable(verbose=True)
        self._cb_called = False
        snapshot_pub.publish(build_msg(DenseLaserSnapshot, 2))
        laser_image_pub.publish(build_msg(Image, 2))
        #image_features_pub.publish(build_msg(CalibrationPattern, 1))
        joint_features_pub.publish(build_msg(JointStateCalibrationPattern, 2))
        duration_pub.publish(build_msg(IntervalStamped, 2))
        time.sleep(1.0)
        self.assertFalse(self._cb_called)

        # Only send a few of the messages again. The concise synchronizer should find a match
        manager.enable(verbose=False)
        self._cb_called = False
        snapshot_pub.publish(build_msg(DenseLaserSnapshot, 3))
        laser_image_pub.publish(build_msg(Image, 3))
        #image_features_pub.publish(build_msg(CalibrationPattern, 1))
        joint_features_pub.publish(build_msg(JointStateCalibrationPattern, 3))
        duration_pub.publish(build_msg(IntervalStamped, 3))
        time.sleep(1.0)
        self.assertTrue(self._cb_called)

    def callback(self, cam_id, msg, interval_start, interval_end):
        print "Calling user callback"
        self._cb_called = True
        self._cb_cam_id = cam_id
        self._cb_msg = msg


if __name__ == '__main__':
    import rostest
    rospy.init_node("test_node")
    rostest.unitrun('pr2_calibration_executive', 'test_cam_manager',   TestCamManager)
    rostest.unitrun('pr2_calibration_executive', 'test_chain_manager', TestChainManager)
    rostest.unitrun('pr2_calibration_executive', 'test_laser_manager', TestLaserManager)
