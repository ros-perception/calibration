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
import threading

import rospy
from calibration_msgs.msg import *
import sensor_msgs.msg

import message_filters


class ChainManager:
    def __init__(self, chain_id, callback, *cb_args):
        self._chain_id = chain_id
        self._callback = callback
        self._cb_args = cb_args
        self._mode = "off"

        self._lock = threading.Lock()

        self._chain_sub = rospy.Subscriber(chain_id + "/chain_state", sensor_msgs.msg.JointState, self.callback)

    def callback(self, msg):
        self._lock.acquire()
        if self._mode is "on":
            # Populate measurement message
            m_msg = ChainMeasurement()
            m_msg.header.stamp = msg.header.stamp
            m_msg.chain_id = self._chain_id
            m_msg.chain_state = msg
            self._callback(self._chain_id, m_msg, *self._cb_args)
        self._lock.release()

    def enable(self):
        self._lock.acquire()
        self._mode = "on"
        self._lock.release()

    def disable(self):
        self._lock.acquire()
        self._mode = "off"
        self._lock.release()

class CamManager:
    def __init__(self, cam_id, callback, *cb_args):
        self._cam_id = cam_id
        self._callback = callback
        self._cb_args = cb_args
        self._mode = "off"
        
        self._lock = threading.Lock()

        # How do I specify a queue size of 1?
        self._cam_info_sub   = message_filters.Subscriber(cam_id + "/camera_info", sensor_msgs.msg.CameraInfo)
        self._features_sub   = message_filters.Subscriber(cam_id + "/features",    calibration_msgs.msg.CalibrationPattern)
        self._image_sub      = message_filters.Subscriber(cam_id + "/image",       sensor_msgs.msg.Image)
        self._image_rect_sub = message_filters.Subscriber(cam_id + "/image_rect",  sensor_msgs.msg.Image)
        self._verbose_sync = message_filters.TimeSynchronizer([self._cam_info_sub, self._features_sub, self._image_sub, self._image_rect_sub], 10)
        self._verbose_sync.registerCallback(self.verbose_callback)
        self._minimal_sync = message_filters.TimeSynchronizer([self._cam_info_sub, self._features_sub], 10)
        self._minimal_sync.registerCallback(self.minimal_callback)

    def verbose_callback(self, cam_info, features, image, image_rect):
        self._lock.acquire()
        if self._mode is "verbose":
            # Populate measurement message
            msg = CameraMeasurement()
            msg.header.stamp = cam_info.header.stamp
            msg.camera_id = self._cam_id
            msg.image_points = features.image_points
            msg.cam_info = cam_info
            msg.verbose = True
            msg.image = image
            msg.image_rect = image_rect
            msg.features = features
            self._callback(self._cam_id, msg, *self._cb_args)
        self._lock.release()

    def minimal_callback(self, cam_info, features):
        self._lock.acquire()
        if self._mode is "minimal":
            # Populate measurement message
            msg = CameraMeasurement()
            msg.header.stamp = cam_info.header.stamp
            msg.camera_id = self._cam_id
            msg.image_points = features.image_points
            msg.cam_info = cam_info
            msg.verbose = False
            self._callback(self._cam_id, msg, *self._cb_args)
        self._lock.release()

    def enable(self, verbose=False):
        self._lock.acquire()
        if verbose:
            self._mode = "verbose"
        else:
            self._mode = "minimal"
        self._lock.release()

    def disable(self):
        self._lock.acquire()
        self._mode = "off"
        self._lock.release()

class LaserManager:
    def __init__(self, laser_id, callback, *cb_args):
        self._laser_id = laser_id
        self._callback = callback
        self._cb_args = cb_args
        self._mode = "off"

        self._lock = threading.Lock()

        # How do I specify a queue size of 1?
        self._snapshot_sub       = message_filters.Subscriber(laser_id + "/snapshot",       calibration_msgs.msg.DenseLaserSnapshot)
        self._laser_image_sub    = message_filters.Subscriber(laser_id + "/laser_image",    sensor_msgs.msg.Image)
        self._image_features_sub = message_filters.Subscriber(laser_id + "/image_features", calibration_msgs.msg.CalibrationPattern)
        self._joint_features_sub = message_filters.Subscriber(laser_id + "/joint_features", calibration_msgs.msg.JointStateCalibrationPattern)
        self._duration_sub       = message_filters.Subscriber(laser_id + "/laser_interval", calibration_msgs.msg.IntervalStamped)

        self._verbose_sync = message_filters.TimeSynchronizer([self._snapshot_sub,
                                                               self._laser_image_sub,
                                                               self._image_features_sub,
                                                               self._joint_features_sub,
                                                               self._duration_sub], 10)
        self._minimal_sync = message_filters.TimeSynchronizer([self._joint_features_sub,
                                                               self._duration_sub], 10)

        self._verbose_sync.registerCallback(self.verbose_callback)
        self._minimal_sync.registerCallback(self.minimal_callback)

    def verbose_callback(self, snapshot, laser_image, image_features, joint_features, laser_duration):
        self._lock.acquire()
        if self._mode is "verbose":
            # Populate measurement message
            msg = LaserMeasurement()
            msg.laser_id = self._laser_id
            msg.joint_points = joint_features.joint_points
            msg.verbose = True
            msg.snapshot = snapshot
            msg.laser_image = laser_image
            msg.image_features = image_features
            msg.joint_features = joint_features
            self._callback(self._laser_id, msg, laser_duration.interval.start, laser_duration.interval.end, *self._cb_args)
        self._lock.release()

    def minimal_callback(self, joint_features, laser_duration):
        self._lock.acquire()
        if self._mode is "minimal":
            # Populate measurement message
            msg = LaserMeasurement()
            msg.laser_id = self._laser_id
            msg.joint_points = joint_features.joint_points
            msg.verbose = True
            self._callback(self._laser_id, msg, laser_duration.interval.start, laser_duration.interval.end, *self._cb_args)
        self._lock.release()

    def enable(self, verbose=False):
        self._lock.acquire()
        if verbose:
            self._mode = "verbose"
        else:
            self._mode = "minimal"
        self._lock.release()

    def disable(self):
        self._lock.acquire()
        self._mode = "off"
        self._lock.release()

