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
import rospy
import yaml
import sys
import actionlib
import joint_states_settler.msg
import time
import threading

import capture_executive
from capture_executive.config_manager import ConfigManager
from capture_executive.sensor_managers import *
from capture_executive.robot_measurement_cache import RobotMeasurementCache

from calibration_msgs.msg import RobotMeasurement

class CaptureExecutive:
    def __init__(self, config_dir):
        self.cam_config        = yaml.load(open(config_dir + "/cam_config.yaml"))
        self.chain_config      = yaml.load(open(config_dir + "/chain_config.yaml"))
        self.laser_config      = yaml.load(open(config_dir + "/laser_config.yaml"))
        self.controller_config = yaml.load(open(config_dir + "/controller_config.yaml"))
        
        # Not all robots have lasers.... :(
        if self.laser_config == None:
            self.laser_config = dict()

        self.cache = RobotMeasurementCache()
        self.lock = threading.Lock()

        # Specifies if we're currently waiting for a sample
        self.active = False

        # Construct Configuration Manager (manages configuration of nodes in the pipeline)
        self.config_manager = ConfigManager(self.cam_config,
                                            self.chain_config,
                                            self.laser_config,
                                            self.controller_config)

        # Construct a manager for each sensor stream (Don't enable any of them)
        self.cam_managers   = [ (cam_id,   CamManager(  cam_id,   self.add_cam_measurement) )   for cam_id   in self.cam_config.keys() ]
        self.chain_managers = [ (chain_id, ChainManager(chain_id, self.add_chain_measurement) ) for chain_id in self.chain_config.keys() ]
        self.laser_managers = [ (laser_id, LaserManager(laser_id, self.add_laser_measurement) ) for laser_id in self.laser_config.keys() ]

        # Subscribe to topic containing stable intervals
        self.request_interval_sub = rospy.Subscriber("request_interval", calibration_msgs.msg.Interval, self.request_callback)

        # Hardcoded cache sizes. Not sure where these params should live
        # ...


    def capture(self, next_configuration, timeout):
        done = False
        self.m_robot = None

        timeout_time = rospy.Time().now() + timeout

        self.lock.acquire()
        if self.active:
            print "Can't capture since we're already active"
            done = True
        self.lock.release()

        # Set up the pipeline
        self.config_manager.reconfigure(next_configuration)

        time.sleep(2.0)

        # Set up the cache with only the sensors we care about
        cam_ids   = [x["cam_id"]   for x in next_configuration["camera_measurements"]]
        chain_ids = [x["chain_id"] for x in next_configuration["joint_measurements"]]
        try:
            laser_ids = [x["laser_id"] for x in next_configuration["laser_measurements"]]
        except:
            laser_ids = list()
        self.cache.reconfigure(cam_ids, chain_ids, laser_ids)


        print "Setting up sensor managers"
        enable_list = []
        disable_list = []
        # Set up the sensor managers
        for cam_id, cam_manager in self.cam_managers:
            if cam_id in [x["cam_id"] for x in next_configuration["camera_measurements"]]:
                enable_list.append(cam_id)
                cam_manager.enable(True)
            else:
                disable_list.append(cam_id)
                cam_manager.disable()

        for chain_id, chain_manager in self.chain_managers:
            if chain_id in [x["chain_id"] for x in next_configuration["joint_measurements"]]:
                enable_list.append(chain_id)
                chain_manager.enable()
            else:
                disable_list.append(chain_id)
                chain_manager.disable()

        for laser_id, laser_manager in self.laser_managers:
            enabled_lasers = [x["laser_id"] for x in next_configuration["laser_measurements"]]
            if laser_id in enabled_lasers:
                enable_list.append(laser_id)
                laser_manager.enable(True)
            else:
                disable_list.append(laser_id)
                laser_manager.disable()

        print "Enabling"
        for cur_enabled in enable_list:
            print " + %s" % cur_enabled
        print "Disabling"
        for cur_disabled in disable_list:
            print " - %s" % cur_disabled

        self.lock.acquire()
        self.active = True
        self.lock.release()

        # Keep waiting until the request_callback function populates the m_robot msg
        while (not rospy.is_shutdown()) and (not done) and (rospy.Time().now() < timeout_time):
            time.sleep(.1)
            self.lock.acquire()
            if self.m_robot is not None:
                done = True
            self.lock.release()

        # Stop listening to all the sensor streams
        for cam_id, cam_manager in self.cam_managers:
            cam_manager.disable()
        for chain_id, chain_manager in self.chain_managers:
            chain_manager.disable()
        for laser_id, laser_manager in self.laser_managers:
            laser_manager.disable()

        # Turn off the entire pipeline
        self.config_manager.stop()

        # Fill in meta information about the sample
        if self.m_robot is not None:
            self.m_robot.sample_id = next_configuration["sample_id"]
            self.m_robot.target_id = next_configuration["target"]["target_id"]
            self.m_robot.chain_id  = next_configuration["target"]["chain_id"]

        self.lock.acquire()
        self.active = False
        self.lock.release()

        return self.m_robot


    def request_callback(self, msg):

        # See if the interval is big enough to care
        if (msg.end - msg.start) < rospy.Duration(1,0):
            return

        self.lock.acquire()
        if self.active:
            # Do some query into the cache, and possibly stop stuff from running
            self.m_robot = self.cache.request_robot_measurement(msg.start, msg.end)

            # We found a sample, so we can deactive (kind of a race condition, since 'active' triggers capture() to exit... I don't care)
            if self.m_robot is not None:
                self.active = False
        self.lock.release()

    def add_cam_measurement(self, cam_id, msg):
        if len(msg.image_points) > 0:
            self.lock.acquire()
            self.cache.add_cam_measurement(cam_id, msg)
            self.lock.release()

    def add_chain_measurement(self, chain_id, msg):
        self.lock.acquire()
        self.cache.add_chain_measurement(chain_id, msg)
        self.lock.release()

    def add_laser_measurement(self, laser_id, msg, interval_start, interval_end):
        self.lock.acquire()
        print "Adding laser measurement"
        self.cache.add_laser_measurement(laser_id, msg, interval_start, interval_end)
        self.lock.release()

