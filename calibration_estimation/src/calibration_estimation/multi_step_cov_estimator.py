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

import roslib; roslib.load_manifest('calibration_estimation')

import sys
import rospy
import time
import numpy
import rosbag
import yaml
import os.path
import numpy
import math

import stat
import os

from numpy import matrix

from calibration_estimation.cal_bag_helpers import *
from calibration_estimation.urdf_params import UrdfParams
from calibration_estimation.sensors.multi_sensor import MultiSensor
from calibration_estimation.opt_runner import opt_runner

from calibration_estimation.single_transform import angle_axis_to_RPY, RPY_to_angle_axis

def usage():
    rospy.logerr("Not enough arguments")
    print "Usage:"
    print " ./proto1.py [bagfile] [output_dir]"
    sys.exit(0)


def build_sensor_defs(sensors):
    '''
    Given a list of sensor definition dictionaries, merge them into a single dictionary
    '''
    all_sensors_dict = dict()

    #for cur_sensors_file in sensors_dump:
    for cur_sensor_type, cur_sensor_list in sensors.items():
        if cur_sensor_type in ['checkerboards']:
            continue
        for cur_sensor_name, cur_sensor in cur_sensor_list.items():
            cur_sensor['sensor_id'] = cur_sensor_name  # legacy support...
            # We want sensor_ids to be unique. Thus, we should warn the user if there are duplicate block IDs being loaded
            if cur_sensor['sensor_id'] in all_sensors_dict.keys():  # TODO: can we get rid of this?
                rospy.logwarn("Loading a duplicate [%s]. Overwriting previous" % cur_sensor['sensor_id'])
            all_sensors_dict[cur_sensor['sensor_id']] = cur_sensor
            all_sensors_dict[cur_sensor['sensor_id']]['sensor_type'] = cur_sensor_type

    print "The following error sensors have been loaded:\n"
    # We want to sort by the types of blocks
    all_sensor_types = list(set([x['sensor_type'] for x in all_sensors_dict.values()]))

    for cur_sensor_type in all_sensor_types:
        print "  %s sensors" % cur_sensor_type
        cur_sensor_ids = [cur_sensor_id for cur_sensor_id,cur_sensor in all_sensors_dict.items() if cur_sensor['sensor_type'] == cur_sensor_type]
        cur_sensor_ids.sort()
        for cur_sensor_id in cur_sensor_ids:
            print "   - %s" % cur_sensor_id
        print ""

    return all_sensors_dict

def load_calibration_steps(steps_dict):
    # We want to execute the calibration in alphabetical order, based on the key names
    step_keys = steps_dict.keys()
    step_keys.sort()
    step_list = []
    for step_name in step_keys:
        # Add the step name to the dict (since we'll lose this info once we listify)
        steps_dict[step_name]["name"] = step_name
        step_list.append(steps_dict[step_name])
    print "Loaded the calibration steps to execute in the following order:"
    for cur_step in step_list:
        print " - %s" % cur_step['name']
    return step_list

# Verifies that the given filename has the correct permissions.
# Either it shouldn't exist, or it should be writable
def check_file_permissions(filename):
    # If the file doesn't exist, then we're fine
    if not os.path.isfile(filename):
        return True

    statinfo = os.stat(filename);

    # See if the current user owns the file. If so, look at user's write priveleges
    if (os.geteuid() == statinfo.st_uid):
        return (os.stat(filename).st_mode & stat.S_IWUSR) > 0

    # See if the current user's group owns the file. If so, look at groups's write priveleges
    if (os.getgid() == statinfo.st_gid):
        return (os.stat(filename).st_mode & stat.S_IWGRP) > 0

    # Not the owner, nor part of the group, so check the 'other' permissions
    return (os.stat(filename).st_mode & stat.S_IWOTH) > 0


def load_requested_sensors(all_sensors_dict, requested_sensors):
    '''
    Build a sensor dictionary with the subset of sensors that we request
    '''
    all_sensor_types = list(set([x['sensor_type'] for x in all_sensors_dict.values()]))
    cur_sensors = dict([(x,[]) for x in all_sensor_types])
    for requested_sensor_id in requested_sensors:
        # We need to now find requested_sensor_id in our library of sensors
        if requested_sensor_id in all_sensors_dict.keys():
            cur_sensor_type = all_sensors_dict[requested_sensor_id]['sensor_type']
            cur_sensors[cur_sensor_type].append(all_sensors_dict[requested_sensor_id])
        else:
            rospy.logerr("Could not find [%s] in block library. Skipping this block", requested_sensor_id)
    return cur_sensors


def diff(v1, v2, eps = 1e-10):
    ''' Determine the difference in two vectors. '''
    if sum( [ math.fabs(x-y) for x,y in zip(v1, v2) ] ) <= eps:
        return 0
    return 1

# URDF updating -- this should probably go in a different file
def update_transmission(urdf, joint, gearing):
    for transmission in urdf.transmissions.values():
        if transmission.joint == joint:
            transmission.reduction = transmission.reduction * gearing
            return
    print "No transmission found for:", joint

def update_urdf(urdf, calibrated_params):
    ''' Given urdf and calibrated robot_params, updates the URDF. '''
    joints = list()
    axis = list()
    # update each transmission
    for chain in calibrated_params.chains.values():
        joints += chain._active
        axis += numpy.array(chain._axis)[0,:].tolist()
        for joint, gearing in zip(chain._active, chain._gearing):
            if gearing != 1.0:
                update_transmission(urdf, joint, gearing)
    for laser in calibrated_params.tilting_lasers.values():
        joints.append(laser._config['joint'])
        axis.append(5) # TODO: remove this assumption
        if laser._gearing != 1.0:
            update_transmission(urdf, laser._config['joint'], laser._gearing)

    unchanged_joints = [];

    # update each transform (or joint calibration)
    for joint_name in calibrated_params.transforms.keys():
        link_updated = 0
        try:
            updated_link_params = calibrated_params.transforms[joint_name]._config.T.tolist()[0]
            if diff(updated_link_params[0:3],  urdf.joints[joint_name].origin.position):
                print 'Updating xyz for', joint_name, '\n old:', urdf.joints[joint_name].origin.position, '\n new:', updated_link_params[0:3]
                urdf.joints[joint_name].origin.position = updated_link_params[0:3]
                link_updated = 1
            r1 = RPY_to_angle_axis(urdf.joints[joint_name].origin.rotation)
            if diff(r1, updated_link_params[3:6]):
                # TODO: remove assumption that joints are revolute
                if joint_name in joints and urdf.joints[joint_name].calibration != None:
                    cal = urdf.joints[joint_name].calibration 
                    a = axis[joints.index(joint_name)]
                    a = int(a) - 1
                    print 'Updating calibration for', joint_name, 'by', updated_link_params[a]
                    if cal.rising != None:
                        urdf.joints[joint_name].calibration.rising += updated_link_params[a]   
                    if cal.falling != None:
                        urdf.joints[joint_name].calibration.falling += updated_link_params[a]
                    link_updated = 1
                else:
                    rot = angle_axis_to_RPY(updated_link_params[3:6])
                    print 'Updating rpy for', joint_name, '\n old:', urdf.joints[joint_name].origin.rotation, '\n new:', rot
                    urdf.joints[joint_name].origin.rotation = rot
                    link_updated = 1                
        except KeyError:
            print "Joint removed:", joint_name
            link_updated = 1
        if not link_updated:
            unchanged_joints.append( joint_name );
    
    print "The following joints weren't updated: \n", ', '.join(unchanged_joints)
    return urdf

if __name__ == '__main__':
    import time
    xml_time = time.strftime('%Y_%m_%d_%H_%M', time.localtime())
    calibrated_xml = 'robot_calibrated_'+xml_time+'.xml'
    uncalibrated_xml = 'robot_uncalibrated_'+xml_time+'.xml'

    rospy.init_node("multi_step_cov_estimator", anonymous=True)
    print "Starting The Multi Step [Covariance] Estimator Node\n"

    if (len(rospy.myargv()) < 2):
        usage()
    elif (len(rospy.myargv()) < 3):
        bag_filename = rospy.myargv()[1]
        output_dir = "."
    else:
        bag_filename = rospy.myargv()[1]
        output_dir = rospy.myargv()[2]

    print "Using Bagfile: %s\n" % bag_filename
    if not os.path.isfile(bag_filename):
        rospy.logerr("Bagfile does not exist. Exiting...")
        sys.exit(1)

    config_param_name = "calibration_config"
    if not rospy.has_param(config_param_name):
        rospy.logerr("Could not find parameter [%s]. Please populate this namespace with the estimation configuration.", config_param_name)
        sys.exit(1)
    config = rospy.get_param(config_param_name)

    # Process all the sensor definitions that are on the parameter server
    sensors_name = "sensors"
    if sensors_name not in config.keys():
        rospy.logerr("Could not find namespace [%s/%s]. Please populate this namespace with sensors.", (config_param_name, sensors_name))
        sys.exit(1)
    #sensors_dump = [yaml.load(x) for x in config[sensors_name].values()]
    all_sensors_dict = build_sensor_defs(config[sensors_name])
    all_sensor_types = list(set([x['sensor_type'] for x in all_sensors_dict.values()]))

    # Load all the calibration steps.
    step_list = load_calibration_steps(config["cal_steps"])

    # Count how many checkerboard poses we need to track
    sample_skip_list = rospy.get_param('calibration_skip_list', [])
    msg_count = get_robot_measurement_count(bag_filename, sample_skip_list)

    if 'initial_poses' in config.keys():
        previous_pose_guesses = numpy.array(yaml.load(config['initial_poses']))
    else:
        previous_pose_guesses = numpy.zeros([msg_count,6])
        
        # TODO: add alternate methods of defining default poses guesses
        # See https://github.com/ros-perception/calibration/pull/9
        if 'default_floating_initial_pose' in config.keys():
            default_pose = config['default_floating_initial_pose']
            if len(default_pose) != 6:
                print "The 'default_floating_initial_pose' parameter has", len(default_pose), "elements, but it should have 6!"
                sys.exit(-1)
            for p in range(msg_count):
                previous_pose_guesses[p,] = config['default_floating_initial_pose']

    # Check if we can write to all of our output files
    output_filenames = [calibrated_xml]
    for suffix in [".yaml", "_poses.yaml", "_cov.txt"]:
        output_filenames += [output_dir + "/" + cur_step["output_filename"] + suffix for cur_step in step_list]

    valid_list = [check_file_permissions(curfile) for curfile in output_filenames];
    permissions_valid = all(valid_list)
    if not permissions_valid:
        print "Invalid file permissions. You need to be able to write to the following files:"
        print "\n".join([" - " + cur_file for cur_file,cur_valid in zip(output_filenames, valid_list) if not cur_valid])
        sys.exit(-1)

    # Generate robot parameters
    robot_description = get_robot_description(bag_filename)
    robot_params = UrdfParams(robot_description, config)

    # Load all the sensors from the bagfile and config file
    for cur_step in step_list:
        print ""
        print "*"*30
        print "Beginning [%s]" % cur_step["name"]

        # Need to load only the sensors that we're interested in
        cur_sensors = load_requested_sensors(all_sensors_dict, cur_step['sensors'])

        # Load all the sensors from bag
        multisensors = get_multisensors(bag_filename, cur_sensors, sample_skip_list)

        # Display sensor count statistics
        print "Executing step with the following Sensors:"
        # Iterate over sensor definitions for this step
        for cur_sensor_type, cur_sensor_list in cur_sensors.items():
            print "  %s Sensors:" % cur_sensor_type
            cur_sensor_ids = [cur_sensor['sensor_id'] for cur_sensor in cur_sensor_list]
            cur_sensor_ids.sort()
            for cur_sensor_id in cur_sensor_ids:
                counts = [ len([s for s in ms.sensors if s.sensor_id == cur_sensor_id]) for ms in multisensors]
                count = sum(counts)
                print "   - %s (%u)" % (cur_sensor_id, count)
            print ""

        print "Sensor breakdown (By Sample):"
        for k,ms in zip(range(len(multisensors)), multisensors):
            print " % 2u) %s" % (k, ", ".join([s.sensor_id for s in ms.sensors]))

        print "Pose Guesses:\n", previous_pose_guesses

        if len(multisensors) == 0:
            rospy.logwarn("No error blocks were generated for this optimization step. Skipping this step.  This will result in a miscalibrated sensor")
            output_dict = robot_params.params_to_config(robot_params.deflate())
            output_poses = previous_pose_guesses
        else:
            free_dict = yaml.load(cur_step["free_params"])
            use_cov = cur_step['use_cov']
            if use_cov:
                print "Executing step with covariance calculations"
            else:
                print "Executing step without covariance calculations"
            output_dict, output_poses, J = opt_runner(robot_params, previous_pose_guesses, free_dict, multisensors, use_cov)

        # Dump results to file
        out_f = open(output_dir + "/" + cur_step["output_filename"] + ".yaml", 'w')
        yaml.dump(output_dict, out_f)
        out_f.close()

        out_f = open(output_dir + "/" + cur_step["output_filename"] + "_poses.yaml", 'w')
        yaml.dump([list([float(x) for x in pose]) for pose in list(output_poses)], out_f)
        out_f.close()

        cov_x = matrix(J).T * matrix(J)
        numpy.savetxt(output_dir + "/" + cur_step["output_filename"] + "_cov.txt", cov_x, fmt="% 9.3f")

        previous_pose_guesses = output_poses

    # write original urdf so you can do a diff later
    rospy.loginfo('Writing original urdf to %s', uncalibrated_xml)
    outfile = open(uncalibrated_xml, 'w')
    outfile.write( robot_params.get_clean_urdf().to_xml() )
    outfile.close()

    #update urdf
    urdf = update_urdf(robot_params.get_clean_urdf(), robot_params)

    # write out to URDF
    outfile = open(calibrated_xml, 'w')
    rospy.loginfo('Writing updates to %s', calibrated_xml)
    outfile.write( urdf.to_xml() )
    outfile.close()

    outfile = open('latest_calibrated_xml', 'w')
    outfile.write( calibrated_xml )
    outfile.close()
