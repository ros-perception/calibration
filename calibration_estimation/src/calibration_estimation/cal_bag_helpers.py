#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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

import roslib; roslib.load_manifest('calibration_estimation')
import rosbag


def get_cam_info(bag_filename, cam_name):
    """ Look for cam info in the calibration bagfile """
    cam_info = None
    bag = rosbag.Bag(bag_filename)
    for topic, msg, t in bag.read_messages():
        if topic == "robot_measurement":
            for cam_measurement in msg.M_cam:
                if cam_measurement.camera_id == cam_name:
                    print "Found a sample with camera [%s] in it" % cam_measurement.camera_id
                    cam_info = cam_measurement.cam_info
                    break
            if cam_info != None:
                break
    return cam_info


def get_robot_description(bag_filename):
    """ Get the robot description out of a bagfile """
    bag = rosbag.Bag(bag_filename)
    for topic, msg, t in bag.read_messages(topics=['robot_description']):
        if topic == 'robot_description':
            bag.close()
            return msg.data
    bag.close()
    return ""


def get_robot_measurement_count(bag_filename):
    """ Get the number of measurements in our bagfile """
    msg_count = 0
    bag = rosbag.Bag(bag_filename)
    for topic, msg, t in bag.read_messages(topics=['robot_measurement']):
        if topic == 'robot_measurement':
            msg_count+=1
    bag.close()
    return msg_count

