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
from calibration_estimation.sensors.camera_chain_sensor import CameraChainBundler, CameraChainSensor

from calibration_msgs.msg import *
from sensor_msgs.msg import JointState, CameraInfo

from calibration_estimation.single_transform import SingleTransform
from calibration_estimation.joint_chain import JointChain
from calibration_estimation.camera import RectifiedCamera
from calibration_estimation.tilting_laser import TiltingLaser
from calibration_estimation.full_chain import FullChainCalcBlock
from calibration_estimation.checkerboard import Checkerboard

from numpy import *

def loadConfigList():

    config_yaml = '''
- camera_id: camA
  chain:
    before_chain: [transformA]
    chain_id:     chainA
    link_num:     1
    after_chain:  [transformB]
- camera_id: camB
  chain:
    before_chain: [transformA]
    chain_id:     chainB
    link_num:     1
    after_chain:  [transformB]
'''
    config_dict = yaml.load(config_yaml)

    return config_dict

class TestCameraChainBundler(unittest.TestCase):
    def test_basic_match(self):
        config_list = loadConfigList()

        bundler = CameraChainBundler(config_list)

        M_robot = RobotMeasurement( target_id = "targetA",
                                    chain_id = "chainB",
                                    M_cam   = [ CameraMeasurement(camera_id="camA")],
                                    M_chain = [ ChainMeasurement(chain_id="chainA"),
                                                ChainMeasurement(chain_id="chainB") ])

        blocks = bundler.build_blocks(M_robot)

        self.assertEqual( len(blocks), 1)
        block = blocks[0]
        self.assertEqual( block._M_cam.camera_id,  "camA"  )
        self.assertEqual( block._M_chain.chain_id, "chainA")

    def test_basic_no_match(self):
        config_list = loadConfigList()

        bundler = CameraChainBundler(config_list)

        M_robot = RobotMeasurement( target_id = "targetA",
                                    chain_id = "chainA",
                                    M_cam   = [ CameraMeasurement(camera_id="camB")],
                                    M_chain = [ ChainMeasurement(chain_id="chainA")] )

        blocks = bundler.build_blocks(M_robot)

        self.assertEqual( len(blocks), 0)


from calibration_estimation.robot_params import RobotParams

class TestCameraChainSensor(unittest.TestCase):
    def load(self):
        config = yaml.load('''
            camera_id: camA
            chain:
              before_chain: [transformA]
              chain_id:     chainA
              link_num:     1
              after_chain:  [transformB]
            ''')

        robot_params = RobotParams()
        robot_params.configure( yaml.load('''
            chains:
              chainA:
                transforms:
                  - [ 1, 0, 0, 0, 0, 0 ]
                axis: [6]
                gearing: [1]
                cov:
                  joint_angles: [1.0]
              chainB:
                transforms:
                  - [ 2, 0, 0, 0, 0, 0 ]
                axis: [6]
                gearing: [1]
                cov:
                  joint_angles: [1.0]
            tilting_lasers: {}
            rectified_cams:
              camA:
                baseline_shift: 0.0
                f_shift: 0.0
                cx_shift: 0.0
                cy_shift: 0.0
                cov: {u: 1.0, v: 1.0}
            transforms:
                transformA: [0, 0, 0, 0, 0, 0]
                transformB: [0, 0, 0, 0, 0, 0]
                transformC: [0, 0, 0, 0, 0, 0]
                transformD: [0, 0, 0, 0, 0, 0]
            checkerboards: {}
            ''' ) )

        P = [ 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0 ]

        return config, robot_params, P

    def test_cov(self):
        config, robot_params, P = self.load()

        camera_points      = [ ImagePoint(0, 0),
                               ImagePoint(1, 0) ]

        sensor = CameraChainSensor(config,
                                   CameraMeasurement(camera_id="camA",
                                                     cam_info=CameraInfo(P=P),
                                                     image_points=camera_points),
                                   ChainMeasurement(chain_id="chainA",
                                                    chain_state=JointState(position=[0])) )
        sensor.update_config(robot_params)

        target = matrix([[1, 2],
                         [0, 0],
                         [1, 1],
                         [1, 1]])

        cov = sensor.compute_cov(target)
        print "\ncov:\n", cov

        self.assertAlmostEqual(cov[0,0], 1.0, 6)
        self.assertAlmostEqual(cov[1,0], 0.0, 6)
        self.assertAlmostEqual(cov[2,0], 0.0, 6)
        self.assertAlmostEqual(cov[3,0], 0.0, 6)
        self.assertAlmostEqual(cov[1,1], 2.0, 6)
        self.assertAlmostEqual(cov[3,3], 5.0, 6)

    def test_update(self):
        config, robot_params, P = self.load()

        camera_points      = [ ImagePoint(0, 0),
                               ImagePoint(1, 0),
                               ImagePoint(0, 1),
                               ImagePoint(1, 1) ]

        block = CameraChainSensor(config,
                                  CameraMeasurement(camera_id="camA",
                                                    cam_info=CameraInfo(P=P),
                                                    image_points=camera_points),
                                  ChainMeasurement(chain_id="chainA",
                                                   chain_state=JointState(position=[0])) )
        block.update_config(robot_params)

        target = matrix([[1, 2, 1, 2],
                         [0, 0, 1, 1],
                         [1, 1, 1, 1],
                         [1, 1, 1, 1]])

        h = block.compute_expected(target)
        z = block.get_measurement()
        r = block.compute_residual(target)

        self.assertAlmostEqual(linalg.norm( h - matrix( [ [0,1,0,1],
                                                          [0,0,1,1] ] ).T), 0.0, 6)
        self.assertAlmostEqual(linalg.norm( z - matrix( [ [0,1,0,1],
                                                          [0,0,1,1] ] ).T), 0.0, 6)
        self.assertAlmostEqual(linalg.norm( r ), 0.0, 6)

        # Test Sparsity
        sparsity = block.build_sparsity_dict()
        self.assertEqual(sparsity['transforms']['transformA'], [1,1,1,1,1,1])
        self.assertEqual(sparsity['transforms']['transformB'], [1,1,1,1,1,1])
        self.assertEqual(sparsity['chains']['chainA']['transforms'], [[1,1,1,1,1,1]])
        self.assertEqual(sparsity['rectified_cams']['camA']['baseline_shift'], 1)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('calibration_estimation', 'test_CameraChainBundler', TestCameraChainBundler, coverage_packages=['calibration_estimation.blocks.camera_chain'])
    rostest.unitrun('calibration_estimation', 'test_CameraChainSensor',  TestCameraChainSensor,  coverage_packages=['calibration_estimation.blocks.camera_chain'])
