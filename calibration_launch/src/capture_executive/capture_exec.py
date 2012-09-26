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
import os
import string

import capture_executive
from capture_executive.config_manager import ConfigManager
from capture_executive.sensor_managers import *
from capture_executive.robot_measurement_cache import RobotMeasurementCache

from calibration_msgs.msg import RobotMeasurement

# TODO temporary hack until urdf_python is released
roslib.load_manifest('calibration_estimation')
from urdf_python.urdf import *


class CaptureExecutive:
    def __init__(self, config_dir, system, robot_desc, output_debug=False):
        # Load configs
        self.cam_config        = yaml.load(open(config_dir + "/cam_config.yaml"))
        self.chain_config      = yaml.load(open(config_dir + "/chain_config.yaml"))
        self.laser_config      = yaml.load(open(config_dir + "/laser_config.yaml"))
        self.controller_config = yaml.load(open(config_dir + "/controller_config.yaml"))
        # Not all robots have lasers.... :(
        if self.laser_config == None:
            self.laser_config = dict()
        # Debug mode makes bag files huge...
        self.output_debug = output_debug

        # Error message from sample capture
        self.message = None

        # Status message from interval computation
        self.interval_status = None

        # parse urdf and get list of links
        links = URDF().parse(robot_desc).links.keys()

        # load system config
        system = yaml.load(open(system))
        
        # remove cams that are not on urdf
        for cam in self.cam_config.keys():
            try:
                link = system['sensors']['rectified_cams'][cam]['frame_id']
                if not link in links:
                    print 'URDF does not contain link [%s]. Removing camera [%s]' % (link, cam)
                    del self.cam_config[cam]
            except:
                print 'System description does not contain camera [%s]' % cam
                del self.cam_config[cam]

        # remove arms that are not on urdf
        for chain in self.chain_config.keys():
            try:
                link = system['sensors']['chains'][chain]['tip']
                if not link in links:
                    print 'URDF does not contain link [%s]. Removing chain [%s]' % (link, chain)
                    del self.chain_config[chain]
            except:
                print 'System description does not contain chain [%s]' % chain
                del self.chain_config[chain]

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
        self.cam_managers  = [ (cam_id,   CamManager(  cam_id,   self.add_cam_measurement) )   for cam_id   in self.cam_config.keys() ]
        self.chain_managers = [ (chain_id, ChainManager(chain_id, self.add_chain_measurement) ) for chain_id in self.chain_config.keys() ]
        self.laser_managers = [ (laser_id, LaserManager(laser_id, self.add_laser_measurement) ) for laser_id in self.laser_config.keys() ]

        # Subscribe to topic containing stable intervals
        self.request_interval_sub = rospy.Subscriber("intersected_interval", calibration_msgs.msg.Interval, self.request_callback)

        # Subscribe to topic containing stable intervals
        self.interval_status_sub = rospy.Subscriber(
              "intersected_interval_status",
              calibration_msgs.msg.IntervalStatus, self.status_callback)

        # Hardcoded cache sizes. Not sure where these params should live
        # ...

    def capture(self, next_configuration, timeout):
        done = False
        self.m_robot = None
        self.message = None
        self.interval_status = None

        timeout_time = rospy.Time().now() + timeout

        self.lock.acquire()
        if self.active:
            rospy.logerr("Can't capture since we're already active")
            done = True
        self.lock.release()

        camera_measurements = next_configuration["camera_measurements"]
        next_configuration["camera_measurements"] = []

        for (i, cam) in enumerate(camera_measurements):
            if self.cam_config.has_key(cam["cam_id"]):
                next_configuration["camera_measurements"].append(cam)
            else:
                rospy.logdebug("Not capturing measurement for camera: %s"%(cam["cam_id"]))

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

        rospy.logdebug("Setting up sensor managers")
        enable_list = []
        disable_list = []
        # Set up the sensor managers
        for cam_id, cam_manager in self.cam_managers:
            if cam_id in [x["cam_id"] for x in next_configuration["camera_measurements"]]:
                enable_list.append(cam_id)
                cam_manager.enable(self.output_debug)
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
                laser_manager.enable(self.output_debug)
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
        elif self.interval_status is not None:
            total = 0
            bad_sensors = []
            l = len(self.interval_status.names)
            if l != len(self.interval_status.yeild_rates):
               rospy.logerr("Invalid interval status message; names and yeild rates have different lengths")
               l = min(l, len(self.interval_status.yeild_rates))
            # analyze status message
            for i in range(l):
               if self.interval_status.yeild_rates[i] == 0.0:
                  bad_sensors.append(self.interval_status.names[i])
               total += self.interval_status.yeild_rates[i]

            # if we didn't get any samples from some sensors, complain
            if len(bad_sensors) > 0:
               print "Didn't get good data from %s"%(', '.join(bad_sensors))
            else:
               # if we got data from all of our sensors, complain about the
               # ones that were below the mean (heuristic)
               avg = total / l
               for i in range(l):
                  if self.interval_status.yeild_rates[i] <= avg:
                     bad_sensors.append(self.interval_status.names[i])
               print "%s may be performing poorly"%(", ".join(bad_sensors))
        elif self.message is not None:
            print self.message

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
            m = self.cache.request_robot_measurement(msg.start, msg.end)
            if isinstance(m, basestring):
               self.message = m
            else:
               self.m_robot = m
            # We found a sample, so we can deactive (kind of a race condition, since 'active' triggers capture() to exit... I don't care)
            if self.m_robot is not None:
                self.active = False
        self.lock.release()

    def status_callback(self, msg):
        self.lock.acquire()
        if self.active:
            self.interval_status = msg
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
        self.cache.add_laser_measurement(laser_id, msg, interval_start, interval_end)
        self.lock.release()

if __name__=='__main__':
    rospy.init_node("capture_executive_node")

    rospy.logdebug("Starting executive...")
    rospy.sleep(5.0)

    # TODO: convert to parameters?
    samples_dir = rospy.myargv()[1]
    config_dir = rospy.myargv()[2]
    system = rospy.myargv()[3]

    try:
        robot_description = rospy.get_param('robot_description')
    except:
        rospy.logfatal('robot_description not set, exiting')
        sys.exit(-1)

    executive = CaptureExecutive(config_dir, system, robot_description)
    time.sleep(1.0)

    # setup our samples
    sample_steps = list()
    sample_names = dict()
    sample_options = dict()
    sample_success = dict()
    sample_failure = dict()
    for directory in os.listdir(samples_dir):
        try:
            sample_options[directory] = yaml.load(open(samples_dir + '/' + directory + '/config.yaml'))
            sample_steps.append(directory)
        except IOError:
            continue
        sample_names[directory] = [x for x in os.listdir(samples_dir + '/' + directory + '/') if '.yaml' in x and x != 'config.yaml']
        sample_names[directory].sort()
        sample_success[directory] = 0
        sample_failure[directory] = 0
    sample_steps.sort()

    for step in sample_steps:
        rospy.logdebug("%s Samples: \n - %s" % (sample_options[step]['group'], "\n - ".join(sample_names[step])))

    pub = rospy.Publisher("robot_measurement", RobotMeasurement)

    try:
        for step in sample_steps:
            keep_collecting = True
            full_paths = [samples_dir + '/' + step + '/' + x for x in sample_names[step] ]
            cur_config = yaml.load(open(full_paths[0]))
            m_robot = executive.capture(cur_config, rospy.Duration(0.01))
            while not rospy.is_shutdown() and keep_collecting:
                print
                print sample_options[step]["prompt"]
                print "Press <enter> to continue, type N to exit this step"
                resp = raw_input(">>>")
                if string.upper(resp) == "N":
                    print sample_options[step]["finish"]
                    keep_collecting = False
                else:
                    for cur_sample_path in full_paths:
                        print "On %s sample [%s]" % (sample_options[step]["group"], cur_sample_path)
                        cur_config = yaml.load(open(cur_sample_path))
                        m_robot = executive.capture(cur_config, rospy.Duration(40))
                        if m_robot is None:
                            print "--------------- Failed To Capture a %s Sample -----------------" % sample_options[step]["group"]
                            if not sample_options[step]["repeat"]:
                                sample_failure[step] += 1
                        else:
                            print "++++++++++++++ Successfully Captured a %s Sample ++++++++++++++" % sample_options[step]["group"]
                            sample_success[step] += 1
                            pub.publish(m_robot)
                        print "Succeeded on %u %s samples" % (sample_success[step], sample_options[step]["group"])
                        if rospy.is_shutdown():
                            break
                    keep_collecting = sample_options[step]["repeat"]
    except EOFError:
        print "Exiting"    

    time.sleep(1)

    print "Calibration data collection has completed!"
    for step in sample_steps:
        if sample_options[step]["repeat"]:
            print "%s Samples: %u" % (sample_options[step]["group"], sample_success[step])
        else:
            print "%s Samples: %u/%u" % (sample_options[step]["group"], sample_success[step], sample_success[step] + sample_failure[step])
    print ""
    print "You can now kill this node, along with any other calibration nodes that are running."


