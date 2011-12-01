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


import roslib; roslib.load_manifest('calibration_estimation')

import sys
import unittest
import rospy
import numpy
import yaml
from calibration_estimation.robot_params import RobotParams
from numpy import *

def loadConfigDict():

    config_yaml = '''
chains:
  chain1:
    transforms:
    - [0, 1, 2, 3, 4, 5]
    - [6, 7, 8, 9, 10, 11]
    axis: [61, 62]
    gearing: [71,72]
    cov:
      joint_angles: [73,74]
  chain2:
    transforms:
    - [12, 13, 15, 15, 15, 15]
    - [14, 15, 16, 17, 0, 0]
    - [18, 19, 20, 21, 0, 0]
    axis: [63, 64, 65]
    gearing: [22,23,24]
    cov:
      joint_angles: [25,26,27]

tilting_lasers:
  laserA:
    before_joint: [0, 0, 0, 0, 0, 0]
    after_joint:  [0, 0, 0, 0, 0, 0]
    gearing: 1

transforms:
  transformA: [0, 0, 0, 0, 0, 0]

rectified_cams:
  camA:
    baseline_shift: 0.0
    f_shift: 0.0
    cx_shift: 0.0
    cy_shift: 0.0
    cov:
      u: 1
      v: 2

checkerboards:
  boardA:
    corners_x: 2
    corners_y: 3
    spacing_x: 10
    spacing_y: 20
'''
    config_dict = yaml.load(config_yaml)

    #import code; code.interact(local=locals())
    return config_dict

def loadParamVec():
    params = [ # DH Chains
                 # Chain 2
                   0, 0, 0, 0, 0, 0,
                   0,10, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0,
                 # Chain 1
                 -10, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0,
               # Tilting Lasers
                 # Laser A
                   0,20, 0, 0, 0, 0,
                   0, 0,30, 0, 0, 0,
                   1,
               # Transforms
                 # transformA
                  40, 0, 0, 0, 0, 0,
               # Rectified Cams
                 # Cam A
                   4, 0, 0, 0,
               # Checkerboards
                 # Board A
                   30, 40]
    param_vec = matrix( params, float).T
    return param_vec

def loadFreeDict():
    free_yaml = '''
chains:
  chain1:
    transforms:
    - [0, 0, 0, 1, 0, 0]
    - [0, 0, 0, 0, 0, 0]
    axis: [0,0]
    gearing: [0,0]
  chain2:
    transforms:
    - [ 1, 0, 0, 0, 0, 0]
    - [ 0, 0, 0, 0, 0, 0]
    - [ 0, 0, 0, 0, 0, 0]
    axis: [0,0,0]
    gearing: [0, 0, 0]

tilting_lasers:
  laserA:
    before_joint: [0, 0, 1, 0, 0, 0]
    after_joint:  [0, 0, 0, 0, 0, 0]
    gearing: 0

transforms:
  transformA: [0, 0, 1, 0, 0, 0]

rectified_cams:
  camA:
    baseline_shift: 0
    f_shift: 0
    cx_shift: 0
    cy_shift: 0

checkerboards:
  boardA:
    spacing_x: 1
    spacing_y: 0
'''
    free_dict = yaml.load(free_yaml)

    #import code; code.interact(local=locals())
    return free_dict


class TestRobotParams(unittest.TestCase):
    def test_configure(self):
        robot_params = RobotParams()

        robot_params.configure(loadConfigDict())

        print "chain1 start: %u" % robot_params.chains["chain1"].start
        print "chain1 end:   %u" % robot_params.chains["chain1"].end

        print "laser start: %u" % robot_params.tilting_lasers["laserA"].start
        print "laser end:   %u" % robot_params.tilting_lasers["laserA"].end

        print "transform start: %u" % robot_params.transforms["transformA"].start
        print "transform end:   %u" % robot_params.transforms["transformA"].end

        print "cam start: %u" % robot_params.rectified_cams["camA"].start
        print "cam end:   %u" % robot_params.rectified_cams["camA"].end

        # ****** Joint Chains ******
        self.assertEqual(robot_params.chains["chain1"].start, 21)
        self.assertEqual(robot_params.chains["chain1"].end,   35)

        self.assertEqual(robot_params.chains["chain2"].start,  0)
        self.assertEqual(robot_params.chains["chain2"].end,   21)

        # ****** Tilting Lasers ******
        self.assertEqual(robot_params.tilting_lasers["laserA"].start, 35)
        self.assertEqual(robot_params.tilting_lasers["laserA"].end,   48)

        # ****** Transforms ******
        self.assertEqual(robot_params.transforms["transformA"].start, 48)
        self.assertEqual(robot_params.transforms["transformA"].end,   54)

        # ****** Rectified Cams ******
        self.assertEqual(robot_params.rectified_cams["camA"].start, 54)
        self.assertEqual(robot_params.rectified_cams["camA"].end,   58)

        # ****** Checkerboards ******
        self.assertEqual(robot_params.checkerboards["boardA"].start, 58)
        self.assertEqual(robot_params.checkerboards["boardA"].end,   60)

    def test_inflate(self):
        robot_params = RobotParams()
        robot_params.configure(loadConfigDict())
        robot_params.inflate(loadParamVec())

        self.assertEqual(robot_params.chains["chain1"]._config[0,0], -10)
        self.assertEqual(robot_params.chains["chain2"]._config[1,1],  10)
        self.assertEqual(robot_params.tilting_lasers["laserA"]._before_joint.transform[1,3], 20)
        self.assertEqual(robot_params.tilting_lasers["laserA"]._after_joint.transform[2,3],  30)
        self.assertEqual(robot_params.transforms["transformA"].transform[0,3],  40)
        self.assertEqual(robot_params.rectified_cams["camA"]._config['baseline_shift'],  4)
        self.assertEqual(robot_params.checkerboards["boardA"]._spacing_x,  30)

    def test_params_to_config(self):
        robot_params = RobotParams()
        robot_params.configure(loadConfigDict())
        config = robot_params.params_to_config(loadParamVec())
        self.assertAlmostEqual(config["chains"]["chain2"]['transforms'][1][1], 10, 6)
        self.assertAlmostEqual(config["rectified_cams"]["camA"]["baseline_shift"], 4, 6)
        self.assertAlmostEqual(config["tilting_lasers"]["laserA"]["before_joint"][1], 20, 6)
        self.assertAlmostEqual(config["checkerboards"]["boardA"]["spacing_y"], 40)


    def test_free(self):
        robot_params = RobotParams()
        robot_params.configure(loadConfigDict())
        free_list = robot_params.calc_free(loadFreeDict())
        print free_list
        self.assertEqual(free_list[0], True)
        self.assertEqual(free_list[1], False)
        self.assertEqual(free_list[24], True)
        self.assertEqual(free_list[19], False)
        self.assertEqual(free_list[37], True)

    def test_deflate(self):
        robot_params = RobotParams()
        robot_params.configure(loadConfigDict())

        p = loadParamVec()
        robot_params.inflate(p)

        result = robot_params.deflate()
        self.assertAlmostEqual(numpy.linalg.norm(p - result), 0.0, 6)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('calibration_estimation', 'test_RobotParams', TestRobotParams, coverage_packages=['calibration_estimation.robot_params'])
