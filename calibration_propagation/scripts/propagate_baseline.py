#! /usr/bin/env python

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

# author: Vijay Pradeep

import roslib; roslib.load_manifest('pr2_calibration_propagation')
import sensor_msgs.msg
import sensor_msgs.srv
import rospy
import rosbag

import math
import sys
import yaml

def usage():
    print "Usage: ./baseline_updater camera:=[camera_namespace] [calibration_bagfile] [system_filename] [config_camera_name]"
    print "    camera_namespace: Ros name to prepend onto the call to set_calibration"
    print "    config_filename: yaml file that stores the baseline shift"
    print "    config_camera_name: Name of the camera in the configuration file. Tells script which baseline shift to use"

def main():
    rospy.init_node('baseline_updater', anonymous=True)

    argv = rospy.myargv()

    if len(argv) != 4:
        usage()
        sys.exit(0)

    bag_filename = argv[1]
    system_filename = argv[2]
    config_cam_name = argv[3]


    # ****** Look for cam info in the calibration bagfile ******
    cam_info = None
    bag = rosbag.Bag(bag_filename)
    for topic, msg, t in bag.read_messages():
        if topic == "robot_measurement":
            for cam_measurement in msg.M_cam:
                if cam_measurement.camera_id == config_cam_name:
                    print "Found a sample with camera [%s] in it" % cam_measurement.camera_id
                    cam_info = cam_measurement.cam_info
                    break
            if cam_info != None:
                break

    if cam_info == None:
        print "Could not find a camera of the name [%s]" % config_cam_name
        sys.exit(-1)

    print "Original Projection Matrix:"
    for k in range(3):
        print "  %s" % " ".join(["% 12.5f" % p for p in cam_info.P[k*4:k*4+4]])
    original_baseline = cam_info.P[3]

    # ****** Get the baseline shift from the yaml ******
    system_dict = yaml.load(open(system_filename))
    cam_dict = system_dict['rectified_cams']
    target_cam_dict = cam_dict[config_cam_name]
    baseline_shift = target_cam_dict['baseline_shift']

    print "Baseline Shift:"
    print "  % 12.5f" % baseline_shift

    # ****** Update the baseline ******
    updated_baseline = original_baseline + baseline_shift

    #import code; code.interact(local=locals())
    updated_P = list(cam_info.P)
    updated_P[3] = updated_baseline
    cam_info.P = updated_P

    print "Updated Projection Matrix:"
    for k in range(3):
        print "  %s" % " ".join(["% 12.5f" % p for p in cam_info.P[k*4:k*4+4]])

    # ****** Load new camera info onto camera eeprom ******
    cam_name = rospy.resolve_name("camera")
    set_cal_service_name = cam_name + "/set_camera_info"

    print "Waiting for service [%s] to be available" % set_cal_service_name 
    rospy.wait_for_service(set_cal_service_name)
    print "Writing camera info to camera memory..."
    set_cal_srv = rospy.ServiceProxy(set_cal_service_name, sensor_msgs.srv.SetCameraInfo)
    resp = set_cal_srv(cam_info)
    if resp.success:
        print "Done writing to camera"
    else:
        print "**************** ERROR WRITING TO CAMERA. PLEASE RETRY ***********************"
        sys.exit(-1)

if __name__ == "__main__":
    main()
