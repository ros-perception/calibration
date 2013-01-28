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
import monocam_settler.msg
import image_cb_detector.msg
import laser_cb_detector.msg
import interval_intersection.msg
import time
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Puts the system in a specific configuration, in preparation for collecting a sample
class ConfigManager:
    def __init__(self, cam_config, chain_config, laser_config, controller_config):

        # Set up Cameras
        rospy.logdebug("Constructing Cameras:")
        self._cam_managers = dict()
        for name in cam_config.keys():
            rospy.logdebug("  Constructing CamID [%s]" % name)
            self._cam_managers[name] = CameraConfigManager(name, cam_config[name])

        # Set up Chains
        rospy.logdebug("Constructing Chains:")
        self._chain_managers = dict()
        for name in chain_config.keys():
            rospy.logdebug("  Constructing ChainID [%s]" % name)
            self._chain_managers[name] = ChainConfigManager(name, chain_config[name])

        # Set up lasers
        rospy.logdebug("Constructing Lasers:")
        self._laser_managers = dict()
        for laser_id in laser_config.keys():
            rospy.logdebug("  Constructing LaserID [%s]" % laser_id)
            self._laser_managers[laser_id] = LaserConfigManager(laser_id, laser_config[laser_id])

        # Set up Controllers
        rospy.logdebug("Constructing Controllers:")
        self._controller_managers = dict()
        for controller_id in controller_config.keys():
            rospy.logdebug("  Constructing ControllerID [%s]" % controller_id)
            self._controller_managers[controller_id] = ControllerCmdManager(controller_id, controller_config[controller_id])

        # Set up interval_intersection
        self._intersect_ac = actionlib.SimpleActionClient("interval_intersection_config", interval_intersection.msg.ConfigAction)

    def reconfigure(self, config):
        # Reconfigure Interval Intersection

        intersect_goal = interval_intersection.msg.ConfigGoal();
        try: # handle case where there are no lasers
            intersect_goal.topics = [x['cam_id'] for x in config["camera_measurements"]] + [x['chain_id'] for x in config["joint_measurements"]] + [x['laser_id'] for x in config["laser_measurements"]]
        except KeyError:
            intersect_goal.topics = [x['cam_id'] for x in config["camera_measurements"]] + [x['chain_id'] for x in config["joint_measurements"]]
        self._intersect_ac.send_goal(intersect_goal)

        # Reconfigure the cameras
        rospy.logdebug("Reconfiguring The Cameras")
        for cur_cam in config["camera_measurements"]:
            self._cam_managers[cur_cam["cam_id"]].reconfigure(cur_cam["config"])

        # Reconfigure the chains
        rospy.logdebug("Reconfiguring The Chains")
        for cur_chain in config["joint_measurements"]:
            self._chain_managers[cur_chain["chain_id"]].reconfigure(cur_chain["config"])

        # Reconfigure the lasers
        rospy.logdebug("Reconfiguring The Lasers")
        try:
            for cur_laser in config["laser_measurements"]:
                self._laser_managers[cur_laser["laser_id"]].reconfigure(cur_laser["config"])
        except KeyError:
            pass # if no lasers, nothing to do.

        # Send controller commands
        rospy.logdebug("Sending Controller Commands")
        for cur_controller in config["joint_commands"]:
            self._controller_managers[cur_controller["controller"]].send_command(cur_controller)

    def stop(self):
        pass

# Handles changing the configuration of a joint_states_settler
class ChainConfigManager:
    def __init__(self, chain_id, configs):
        self._chain_id = chain_id
        self._configs = configs
        if not self.validate():
            raise Exception("Invalid chain description")

        # Initialize the ActionClient and state
        self._settler_ac  = actionlib.SimpleActionClient(self._configs["settler_config"], joint_states_settler.msg.ConfigAction)
        self._state = "idle"

    # Check to make sure that config dict has all the necessary fields. TODO: Currently a stub
    def validate(self):
        return True

    # Reconfigure this chain's processing pipeline to the specified configuration. Do nothing if
    # we're already in the correct configuration
    def reconfigure(self, next_config_name):
        if self._state == next_config_name:
            rospy.logdebug("  %s: Configured correctly as [%s]" % (self_.chain_id, self._state))
        else:
            rospy.logdebug("  %s: Need to transition [%s] -> [%s]" % (self._chain_id, self._state, next_config_name))

            next_config   = self._configs["configs"][next_config_name]
            settler_config = next_config["settler"]
            # Convert the settler's configuration from a dictionary to a ROS message
            goal = joint_states_settler.msg.ConfigGoal()
            goal.joint_names = settler_config["joint_names"]
            goal.tolerances  = settler_config["tolerances"]
            goal.max_step    = rospy.Duration(settler_config["max_step"])
            goal.cache_size  = settler_config["cache_size"]

            # Send the goal out
            self._settler_ac.send_goal(goal)

            # TODO: Need to add code that waits for goal to activate


# Handles changing the configuration of the image pipeline associated with a camera
class CameraConfigManager:
    def __init__(self, cam_id, configs):
        self._cam_id = cam_id
        self._configs = configs
        if not self.validate():
            raise Exception("Invalid camera description")

        # Initialize the ActionClients and state
        self._settler_ac   = actionlib.SimpleActionClient(self._configs["settler_config"], monocam_settler.msg.ConfigAction)
        self._led_detector_ac  = actionlib.SimpleActionClient(self._configs["cb_detector_config"], image_cb_detector.msg.ConfigAction)
        self._state = "idle"

    # Check to make sure that config dict has all the necessary fields. TODO: Currently a stub
    def validate(self):
        return True

    # Reconfigure this chain's processing pipeline to the specified configuration. Do nothing if
    # we're already in the correct configuration
    def reconfigure(self, next_config_name):
        if self._state == next_config_name:
            rospy.logdebug("  %s: Configured correctly as [%s]" % (self_.cam_id, self._state))
        else:
            rospy.logdebug("  %s: Need to transition [%s] -> [%s]" % (self._cam_id, self._state, next_config_name))

            next_config   = self._configs["configs"][next_config_name]

            # Send the Settler's Goal
            settler_config = next_config["settler"]
            goal = monocam_settler.msg.ConfigGoal()
            goal.tolerance = settler_config["tolerance"]
            goal.ignore_failures = settler_config["ignore_failures"]
            goal.max_step = rospy.Duration(settler_config["max_step"])
            goal.cache_size  = settler_config["cache_size"]
            self._settler_ac.send_goal(goal)

            # Send the CB Detector Goal
            cb_detector_config = next_config["cb_detector"]
            if not cb_detector_config["active"]:
                rospy.logerr("Can't deal with inactive cb_detector")
            goal = image_cb_detector.msg.ConfigGoal()
            goal.num_x = cb_detector_config["num_x"]
            goal.num_y = cb_detector_config["num_y"]
            goal.spacing_x = 1.0    # Hardcoded garbage value. Should eventually be removed from the msg
            goal.spacing_y = 1.0    # Hardcoded garbage value. Should eventually be removed from the msg
            goal.width_scaling = cb_detector_config["width_scaling"]
            goal.height_scaling = cb_detector_config["height_scaling"]
            goal.subpixel_window = cb_detector_config["subpixel_window"]
            goal.subpixel_zero_zone = cb_detector_config["subpixel_zero_zone"]
            self._led_detector_ac.send_goal(goal)

            # Send the LED Detector Goal
            led_detector_config = next_config["led_detector"]
            if led_detector_config["active"]:
                rospy.logerr("Can't deal with an active led_detector")

            # TODO: Need to add code that waits for goal to activate

# Handles changing the configuration of the image pipeline associated with a camera
class LaserConfigManager:
    def __init__(self, laser_id, configs):
        self._laser_id = laser_id
        self._configs = configs
        if not self.validate():
            raise Exception("Invalid laser description")

        # Initialize the ActionClients and state
        self._settler_ac   = actionlib.SimpleActionClient(self._configs["settler_config"], monocam_settler.msg.ConfigAction)
        self._cb_detector_ac  = actionlib.SimpleActionClient(self._configs["cb_detector_config"], laser_cb_detector.msg.ConfigAction)
        self._state = "idle"

    # Check to make sure that config dict has all the necessary fields. TODO: Currently a stub
    def validate(self):
        return True

    # Reconfigure this chain's processing pipeline to the specified configuration. Do nothing if
    # we're already in the correct configuration
    def reconfigure(self, next_config_name):
        if self._state == next_config_name:
            rospy.logdebug("  %s: Configured correctly as [%s]" % (self_.laser_id, self._state))
        else:
            rospy.logdebug("  %s: Need to transition [%s] -> [%s]" % (self._laser_id, self._state, next_config_name))

            next_config   = self._configs["configs"][next_config_name]

            # Send the Settler's Goal
            settler_config = next_config["settler"]
            goal = monocam_settler.msg.ConfigGoal()
            goal.tolerance = settler_config["tolerance"]
            goal.ignore_failures = settler_config["ignore_failures"]
            goal.max_step = rospy.Duration(settler_config["max_step"])
            goal.cache_size  = settler_config["cache_size"]
            self._settler_ac.send_goal(goal)

            # Send the CB Detector Goal
            cb_detector_config = next_config["cb_detector"]
            if not cb_detector_config["active"]:
                rospy.logerr("Not sure yet how to deal with inactive cb_detector")
            goal = laser_cb_detector.msg.ConfigGoal()
            goal.num_x = cb_detector_config["num_x"]
            goal.num_y = cb_detector_config["num_y"]
            goal.spacing_x = cb_detector_config["spacing_x"]
            goal.spacing_y = cb_detector_config["spacing_y"]
            goal.width_scaling = cb_detector_config["width_scaling"]
            goal.height_scaling = cb_detector_config["height_scaling"]
            goal.subpixel_window = cb_detector_config["subpixel_window"]
            goal.subpixel_zero_zone = cb_detector_config["subpixel_zero_zone"]
            goal.flip_horizontal = cb_detector_config["flip_horizontal"]
            goal.min_intensity = cb_detector_config["min_intensity"]
            goal.max_intensity = cb_detector_config["max_intensity"]
            self._cb_detector_ac.send_goal(goal)

            # TODO: Need to add code that waits for goal to activate



# Handles publishing commands to a trajectory controller
class ControllerCmdManager:
    def __init__(self, controller_id, config):
        self._controller_id = controller_id
        self._config = config
        if not self.validate():
            raise Exception("Invalid chain description")

        # Initialize the Publisher
        self._pub  = rospy.Publisher(self._config["topic"], JointTrajectory)

    # Check to make sure that config dict has all the necessary fields. TODO: Currently a stub
    def validate(self):
        return True

    # Reconfigure this chain's processing pipeline to the specified configuration. Do nothing if
    # we're already in the correct configuration
    def send_command(self, cmd):
        rospy.logdebug("  Sending cmd to controller [%s]"%self._controller_id)
        cmd_msg = JointTrajectory()
        cmd_msg.header.stamp = rospy.Time().now()
        cmd_msg.joint_names = self._config["joint_names"]
        cmd_msg.points = [self._build_segment(x) for x in cmd["segments"]]
        self._pub.publish(cmd_msg)

    def _build_segment(self, config):
        segment = JointTrajectoryPoint()
        segment.positions = config["positions"]
        segment.velocities = [0] * len(config["positions"])
        segment.time_from_start = rospy.Duration(config["duration"])
        return segment



