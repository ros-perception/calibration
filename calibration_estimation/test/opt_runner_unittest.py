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

import roslib; roslib.load_manifest('pr2_calibration_estimation')

import sys
import unittest
import rospy
import numpy
from numpy import matrix, concatenate
import yaml
from pr2_calibration_estimation.sensors.chain_sensor import ChainSensor
from pr2_calibration_estimation.sensors.multi_sensor import MultiSensor
from pr2_calibration_estimation.sensors.tilting_laser_sensor import TiltingLaserSensor
from pr2_calibration_estimation.robot_params import RobotParams
from pr2_calibration_estimation.opt_runner import ErrorCalc

from calibration_msgs.msg import *
from sensor_msgs.msg import JointState

class TestParamJacobian(unittest.TestCase):
    def test_single_sensor(self):
        config = yaml.load('''
                chain_id: chainA
                before_chain: [transformA]
                dh_link_num:  1
                after_chain:  [transformB]
            ''')

        robot_params = RobotParams()
        robot_params.configure( yaml.load('''
            dh_chains:
              chainA:
              - [ 0, 0, 1, 0 ]
              chainB:
              - [ 0, 0, 2, 0 ]
            tilting_lasers: {}
            rectified_cams: {}
            transforms:
                transformA: [0, 0, 0, 0, 0, 0]
                transformB: [0, 0, 0, 0, 0, 0]
            checkerboards:
              boardA:
                corners_x: 2
                corners_y: 2
                spacing_x: 1
                spacing_y: 1
            ''' ) )

        free_dict = yaml.load('''
            dh_chains:
              chainA:
              - [ 1, 0, 1, 0 ]
              chainB:
              - [ 0, 0, 0, 0 ]
            tilting_lasers: {}
            rectified_cams: {}
            transforms:
                transformA: [1, 0, 0, 0, 0, 0]
                transformB: [0, 0, 0, 0, 0, 0]
            checkerboards:
              boardA:
                spacing_x: 1
                spacing_y: 1
            ''')

        chainM = ChainMeasurement( chain_id="chainA", chain_state=JointState(position=[0]))

        sensor = ChainSensor(config, chainM, "boardA" )
        sensor.update_config(robot_params)

        target_pts = matrix([[1, 2, 1, 2],
                             [0, 0, 1, 1],
                             [0, 0, 0, 0],
                             [1, 1, 1, 1]])

        calc = ErrorCalc(robot_params, free_dict, [])

        expanded_param_vec = robot_params.deflate()
        free_list = robot_params.calc_free(free_dict)
        opt_param_mat = expanded_param_vec[numpy.where(free_list), 0].copy()
        opt_param = numpy.array(opt_param_mat)[0]

        J = calc.single_sensor_params_jacobian(opt_param, target_pts, sensor)

        print J

class TestPoseJacobian(unittest.TestCase):
    def test_multisensor_pose_jacobian(self):
        configA = yaml.load('''
                chain_id: chainA
                before_chain: [transformA]
                dh_link_num:  1
                after_chain:  [transformB]
            ''')

        configB = yaml.load('''
                chain_id: chainB
                before_chain: [transformA]
                dh_link_num:  1
                after_chain:  [transformB]
            ''')

        robot_params = RobotParams()
        robot_params.configure( yaml.load('''
            dh_chains:
              chainA:
              - [ 0, 0, 1, 0 ]
              chainB:
              - [ 0, 0, 2, 0 ]
            tilting_lasers: {}
            rectified_cams: {}
            transforms:
                transformA: [0, 0, 0, 0, 0, 0]
                transformB: [0, 0, 0, 0, 0, 0]
            checkerboards:
              boardA:
                corners_x: 2
                corners_y: 2
                spacing_x: 1
                spacing_y: 1
            ''' ) )

        free_dict = yaml.load('''
            dh_chains:
              chainA:
              - [ 1, 0, 0, 0 ]
              chainB:
              - [ 0, 0, 0, 0 ]
            tilting_lasers: {}
            rectified_cams: {}
            transforms:
                transformA: [0, 0, 0, 0, 0, 0]
                transformB: [0, 0, 0, 0, 0, 0]
            checkerboards:
              boardA:
                spacing_x: 0
                spacing_y: 0
            ''')

        sensorA = ChainSensor(configA, ChainMeasurement( chain_id="chainA", chain_state=JointState(position=[0])), "boardA")

        sensorB = ChainSensor(configB, ChainMeasurement( chain_id="chainB", chain_state=JointState(position=[0])), "boardA")

        multisensor = MultiSensor(None)
        multisensor.sensors = [sensorA, sensorB]
        multisensor.checkerboard = "boardA"

        pose_param_vec = numpy.array([0, 0, 0, 0, 0, 0])
        calc = ErrorCalc(robot_params, free_dict, [])

        expanded_param_vec = robot_params.deflate()
        free_list = robot_params.calc_free(free_dict)
        opt_param_mat = expanded_param_vec[numpy.where(free_list), 0].copy()
        opt_param = numpy.array(opt_param_mat)[0]

        J = calc.multisensor_pose_jacobian(opt_param, pose_param_vec, multisensor)

        print J

class TestFullJacobian(unittest.TestCase):
    def test_full_jacobian(self):
        configA = yaml.load('''
                chain_id: chainA
                before_chain: [transformA]
                dh_link_num:  1
                after_chain:  []
            ''')

        configB = yaml.load('''
                chain_id: chainB
                before_chain: []
                dh_link_num:  1
                after_chain:  [transformB]
            ''')

        robot_params = RobotParams()
        robot_params.configure( yaml.load('''
            dh_chains:
              chainA:
              - [ 0, 0, 1, 0 ]
              chainB:
              - [ 0, 0, 2, 0 ]
            tilting_lasers:
              laserB:
                before_joint: [0, 0, 0, 0, 0, 0]
                after_joint:  [0, 0, 0, 0, 0, 0]
            rectified_cams: {}
            transforms:
                transformA: [0, 0, 0, 0, 0, 0]
                transformB: [0, 0, 0, 0, 0, 0]
            checkerboards:
              boardA:
                corners_x: 2
                corners_y: 2
                spacing_x: 1
                spacing_y: 1
              boardB:
                corners_x: 1
                corners_y: 1
                spacing_x: 1
                spacing_y: 1
            ''' ) )

        free_dict = yaml.load('''
            dh_chains:
              chainA:
              - [ 1, 0, 0, 0 ]
              chainB:
              - [ 0, 0, 0, 0 ]
            tilting_lasers:
              laserB:
                before_joint: [1, 0, 0, 0, 0, 0]
                after_joint:  [0, 0, 0, 0, 0, 0]
            rectified_cams: {}
            transforms:
                transformA: [0, 0, 0, 0, 0, 0]
                transformB: [1, 0, 0, 0, 0, 0]
            checkerboards:
              boardA:
                spacing_x: 1
                spacing_y: 1
              boardB:
                spacing_x: 0
                spacing_y: 0
            ''')

        sensorA = ChainSensor(configA, ChainMeasurement( chain_id="chainA", chain_state=JointState(position=[0])), "boardA")
        sensorB = ChainSensor(configB, ChainMeasurement( chain_id="chainB", chain_state=JointState(position=[0])), "boardB")
        laserB  = TiltingLaserSensor({'laser_id':'laserB'}, LaserMeasurement( laser_id="laserB",
                                                                              joint_points=[ JointState(position=[0,0,2]) ] ) )

        multisensorA = MultiSensor(None)
        multisensorA.sensors = [sensorA]
        multisensorA.checkerboard = "boardA"
        multisensorA.update_config(robot_params)

        multisensorB = MultiSensor(None)
        multisensorB.sensors = [sensorB, laserB]
        multisensorB.checkerboard = "boardB"
        multisensorB.update_config(robot_params)

        poseA = numpy.array([1, 0, 0, 0, 0, 0])
        poseB = numpy.array([2, 0, 0, 0, 0, 0])

        calc = ErrorCalc(robot_params, free_dict, [multisensorA, multisensorB])

        expanded_param_vec = robot_params.deflate()
        free_list = robot_params.calc_free(free_dict)
        opt_param_mat = expanded_param_vec[numpy.where(free_list), 0].copy()
        opt_param = numpy.array(opt_param_mat)[0]
        opt_all_vec = concatenate([opt_param, poseA, poseB])

        print "Calculating Sparse"
        J = calc.calculate_jacobian(opt_all_vec)
        numpy.savetxt("J_sparse.txt", J, fmt="% 4.3f")

        #import code; code.interact(local=locals())

        print "Calculating Dense"
        from scipy.optimize.slsqp import approx_jacobian
        J_naive = approx_jacobian(opt_all_vec, calc.calculate_error, 1e-6)
        numpy.savetxt("J_naive.txt", J_naive, fmt="% 4.3f")


        self.assertAlmostEqual(numpy.linalg.norm(J-J_naive), 0.0, 6)

if __name__ == '__main__':
    import rostest
    #rostest.unitrun('pr2_calibration_estimation', 'test_opt_runner', TestParamJacobian, coverage_packages=['pr2_calibration_estimation.opt_runner'])
    #rostest.unitrun('pr2_calibration_estimation', 'test_pose_J', TestPoseJacobian, coverage_packages=['pr2_calibration_estimation.opt_runner'])
    rostest.unitrun('pr2_calibration_estimation', 'test_full_J', TestFullJacobian, coverage_packages=['pr2_calibration_estimation.opt_runner'])
