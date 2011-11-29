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

import calibration_propagation.update_joint as update_joint
import calibration_propagation.process_changelist as process_changelist
import tf.transformations as transformations
import math

def update_urdf(initial_system, calibrated_system, xml_in):
    #find dh_param offsets for all requested dh chains
    dh_offsets = {"right_arm_chain":[],
                  "left_arm_chain":[],
                  "head_chain":[]}

    dh_joint_names = {"right_arm_chain" : ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'],
                      "left_arm_chain"  : ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'],
                      "head_chain"      : ['head_pan_joint', 'head_tilt_joint'] }

    # Check that the chains are in fact in the yaml system config
    chains_to_remove = [x for x in dh_offsets.keys() if x not in initial_system['dh_chains'].keys()];
    print "Need to ignore the following chains:", chains_to_remove
    for chain_to_remove in chains_to_remove:
      del dh_offsets[chain_to_remove]

    print "Computing All dh chain offsets"
    for chain_name in dh_offsets.keys():
        dh_offsets[chain_name] = find_dh_param_offsets(chain_name, initial_system, calibrated_system)
        print "%s offsets:" % chain_name, pplist(dh_offsets[chain_name])

    # Need to be able to lookup the joint offset for each joint
    joint_offsets_list = []
    for chain_name in dh_offsets.keys():
        joint_offsets_list.extend(zip(dh_joint_names[chain_name], dh_offsets[chain_name]))
    joint_offsets = dict(joint_offsets_list)

    #convert transforms to rpy
    transformdict = dict()
    for(name, rotvect) in calibrated_system['transforms'].iteritems():
        floatvect = [mixed_to_float(x) for x in rotvect]
        #print name, pplist(floatvect), angle_axis_to_RPY(floatvect[3:6])
        transformdict[name] = [floatvect[0:3], angle_axis_to_RPY(floatvect[3:6])]

    # Hack in transforms for tilting laser
    floatvect = [mixed_to_float(x) for x in calibrated_system['tilting_lasers']['tilt_laser']['before_joint'] ]
    transformdict['laser_tilt_mount_joint'] =  [floatvect[0:3], angle_axis_to_RPY(floatvect[3:6])]

    #print "Floatvec: ", floatvec
    #print "tuple: ", transformdict['laser_tilt_mount_joint']
    #import code; code.interact(local=locals())
    #assert(False)

    floatvect = [mixed_to_float(x) for x in calibrated_system['tilting_lasers']['tilt_laser']['after_joint'] ]
    transformdict['laser_tilt_joint'] = [floatvect[0:3], angle_axis_to_RPY(floatvect[3:6])]


    # Combine the transforms and joint offsets into a single dict
    joints_dict = dict([(joint_name, [None, None, None]) for joint_name in transformdict.keys() + joint_offsets.keys()])
    for joint_name, val in transformdict.items():
        joints_dict[joint_name][0] = val[0]
        joints_dict[joint_name][1] = val[1]

    for joint_name, offset in joint_offsets.items():
        joints_dict[joint_name][2] = offset

    not_found = joints_dict.keys()
    changelist = []

    for joint_name, val in joints_dict.items():
        cur_cl = update_joint.update_joint(xml_in, joint_name, xyz=val[0], rpy=val[1], ref_shift=val[2])
        if cur_cl is not None:
            not_found.remove(joint_name)
            changelist.extend(cur_cl)

    # Hack to change laser gearing. Assumes that the laser reference position is 0
    reduction_scale = 1.0 / calibrated_system['tilting_lasers']['tilt_laser']['gearing']
    cur_cl = update_joint.update_transmission(xml_in, 'laser_tilt_mount_trans', reduction_scale)
    changelist.extend(cur_cl)

    print "jointnames never found: ", not_found

    for span, result in changelist:
        print "\"%s\" -> \"%s\"" % (xml_in[span[0]:span[1]], result)

    xml_out = process_changelist.process_changelist(changelist, xml_in)

    return xml_out

#pretty-print list to string
def pplist(list):
    zeroed = []
    for x in list:
        if x is None:
            zeroed.append(0.)
        else:
            zeroed.append(x)
    return ' '.join(['%2.3f'%x for x in zeroed])


#return 1 if value1 and value2 are within eps of each other, 0 otherwise
def epsEq(value1, value2, eps = 1e-10):
    if math.fabs(value1-value2) <= eps:
        return 1
    return 0


#convert a float/int/string containing 'pi' to just float
def mixed_to_float(mixed):
    pi = math.pi
    if type(mixed) == str:
        try:
            val = eval(mixed)
        except:
            print >> sys.stderr, "bad value:", mixed, "substituting zero!!\n\n"
            val = 0.
    else:
        val = float(mixed)
    return val


#calculate calibration offsets (whicharm = 'left' or 'right')
def find_dh_param_offsets(chain_name, system_default, system_calibrated):
    offsets = []
    for (default_params, calib_params) in zip(system_default['dh_chains'][chain_name]['dh'], system_calibrated['dh_chains'][chain_name]['dh']):
        #print "default_params[0]:", default_params[0], "calib_params[0]:", calib_params[0]
        diff = mixed_to_float(calib_params[0]) - mixed_to_float(default_params[0])
        if epsEq(diff, 0):
            diff = None
        offsets.append(diff)

    return offsets



#convert from rotation-axis-with-angle-as-magnitude representation to Euler RPY
def angle_axis_to_RPY(vec):
    angle = math.sqrt(sum([vec[i]**2 for i in range(3)]))
    hsa = math.sin(angle/2.)
    if epsEq(angle, 0):
        return (0.,0.,0.)
    quat = [vec[0]/angle*hsa, vec[1]/angle*hsa, vec[2]/angle*hsa, math.cos(angle/2.)]
    rpy = quat_to_rpy(quat)
    return rpy

def rpy_to_quat(rpy):
    return transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], 'sxyz')

def quat_to_rpy(q):
    rpy = transformations.euler_from_quaternion(q, 'sxyz')
    return rpy

def parse_rpy(line):
    return [float(x) for x in line.split("rpy=\"")[1].split("\"")[0].split()]

def parse_xyz(line):
    return [float(x) for x in line.split("xyz=\"")[1].split("\"")[0].split()]
