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

from calibration_estimation.sensors.chain_sensor import ChainBundler, ChainSensor
#from pr2_calibration_estimation.blocks.camera_chain import CameraChainCalcBlock
#from pr2_calibration_estimation.blocks.camera_chain import CameraChainRobotParamsBlock

from calibration_msgs.msg import *
from sensor_msgs.msg import JointState, CameraInfo

from calibration_estimation.single_transform import SingleTransform
from calibration_estimation.joint_chain import JointChain
from calibration_estimation.camera import RectifiedCamera
from calibration_estimation.tilting_laser import TiltingLaser
from calibration_estimation.full_chain import FullChainCalcBlock
from calibration_estimation.checkerboard import Checkerboard
from calibration_estimation.urdf_params import UrdfParams

from numpy import *

def loadSystem():
    urdf = '''
<robot>
  <link name="base_link"/>
  <joint name="j0" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="j0_link"/>
  </joint>
  <link name="j0_link"/>
  <joint name="j1" type="revolute">
    <axis xyz="0 0 1"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="j0_link"/>
    <child link="j1_link"/>
  </joint>
  <link name="j1_link"/>
  <joint name="j2" type="revolute">
    <axis xyz="0 0 1"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="j1_link"/>
    <child link="j2_link"/>
  </joint>
  <link name="j2_link"/>
  <joint name="j3" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="j2_link"/>
    <child link="j3_link"/>
  </joint>
  <link name="j3_link"/>
</robot>
'''
    config = yaml.load('''
sensors:
  chains:
    chainA:
      sensor_id: chainA
      root: j0_link
      tip: j1_link
      cov:
        joint_angles: [1]
      gearing: [1.0]
    chainB:
      sensor_id: chainB
      root: j0_link
      tip: j2_link
      cov:
        joint_angles: [1, 1]
      gearing: [1.0, 1.0]
  rectified_cams: {}
  tilting_lasers: {}
transforms: 
  chainA_cb: [0, 0, 0, 0, 0, 0]
  chainB_cb: [0, 0, 0, 0, 0, 0]
checkerboards:
  boardA:
    corners_x: 2
    corners_y: 2
    spacing_x: 1
    spacing_y: 1
''')
        
    return config["sensors"]["chains"], UrdfParams(urdf, config)


class TestChainBundler(unittest.TestCase):
    def test_basic_match(self):
        config, robot_params = loadSystem()

        bundler = ChainBundler(config.values())

        M_robot = RobotMeasurement( target_id = "targetA",
                                    chain_id = "chainA",
                                    M_cam   = [],
                                    M_chain = [ ChainMeasurement(chain_id="chainA") ])

        blocks = bundler.build_blocks(M_robot)

        self.assertEqual( len(blocks), 1)
        block = blocks[0]
        self.assertEqual( block._M_chain.chain_id, "chainA")
        self.assertEqual( block._target_id, "targetA")

    def test_basic_no_match(self):
        config, robot_params = loadSystem()

        bundler = ChainBundler(config.values())

        M_robot = RobotMeasurement( target_id = "targetA",
                                    chain_id = "chainA",
                                    M_chain = [ ChainMeasurement(chain_id="chainB") ])

        blocks = bundler.build_blocks(M_robot)

        self.assertEqual( len(blocks), 0)

class TestChainSensor(unittest.TestCase):
    def test_cov(self):
        config, robot_params = loadSystem()
        block = ChainSensor(config["chainA"],
                            ChainMeasurement(chain_id="chainA",
                                             chain_state=JointState(position=[0]) ),
                            "boardA")
        block.update_config(robot_params)
        cov = block.compute_cov(None)
        print cov

        self.assertAlmostEqual(cov[0,0], 0.0, 6)
        self.assertAlmostEqual(cov[1,0], 0.0, 6)
        self.assertAlmostEqual(cov[1,1], 1.0, 6)
        self.assertAlmostEqual(cov[4,4], 4.0, 6)

    def test_update1(self):
        config, robot_params = loadSystem()
        block = ChainSensor(config["chainA"],
                            ChainMeasurement(chain_id="chainA",
                                             chain_state=JointState(position=[0]) ),
                            "boardA")
        block.update_config(robot_params)

        target = matrix([[1, 2, 1, 2],
                         [0, 0, 1, 1],
                         [0, 0, 0, 0],
                         [1, 1, 1, 1]])

        h = block.compute_expected(target)
        z = block.get_measurement()
        r = block.compute_residual(target)

        self.assertAlmostEqual(numpy.linalg.norm(target-h), 0.0, 6)

        print "z=\n",z
        print "target=\n",target

        self.assertAlmostEqual(numpy.linalg.norm(target-z), 0.0, 6)
        self.assertAlmostEqual(numpy.linalg.norm(r - numpy.zeros([12])), 0.0, 6)

    def test_update2(self):
        config, robot_params = loadSystem()
        block = ChainSensor(config["chainA"],
                            ChainMeasurement(chain_id="chainA",
                                             chain_state=JointState(position=[numpy.pi / 2.0]) ),
                            "boardA")
        block.update_config(robot_params)

        target = matrix([[0, 0,-1,-1],
                         [1, 2, 1, 2],
                         [0, 0, 0, 0],
                         [1, 1, 1, 1]])

        h = block.compute_expected(target)
        z = block.get_measurement()
        r = block.compute_residual(target)

        self.assertAlmostEqual(numpy.linalg.norm(target-h), 0.0, 6)

        print "z=\n",z
        print "target=\n",target

        self.assertAlmostEqual(numpy.linalg.norm(target-z), 0.0, 6)
        self.assertAlmostEqual(numpy.linalg.norm(r - numpy.zeros([12])), 0.0, 6)

    def test_update3(self):
        config, robot_params = loadSystem()
        block = ChainSensor(config["chainB"],
                            ChainMeasurement(chain_id="chainB",
                                             chain_state=JointState(position=[0.0, 0.0]) ),
                            "boardA")
        block.update_config(robot_params)

        target = matrix([[2, 3, 2, 3],
                         [0, 0, 1, 1],
                         [0, 0, 0, 0],
                         [1, 1, 1, 1]])

        h = block.compute_expected(target)
        z = block.get_measurement()
        r = block.compute_residual(target)

        self.assertAlmostEqual(numpy.linalg.norm(target-h), 0.0, 6)

        print "z=\n",z
        print "target=\n",target

        self.assertAlmostEqual(numpy.linalg.norm(target-z), 0.0, 6)
        self.assertAlmostEqual(numpy.linalg.norm(r - numpy.zeros([12])), 0.0, 6)

    def test_sparsity(self):
        config, robot_params = loadSystem()
        block = ChainSensor(config["chainA"],
                            ChainMeasurement(chain_id="chainA",
                                             chain_state=JointState(position=[numpy.pi / 2.0]) ),
                            "boardA")
        block.update_config(robot_params)
        sparsity = block.build_sparsity_dict()
        self.assertEqual(sparsity['transforms']['j0'], [1,1,1,1,1,1])
        self.assertEqual(sparsity['transforms']['j1'], [1,1,1,1,1,1])
        self.assertEqual(sparsity['chains']['chainA'], {'gearing':[1]})

if __name__ == '__main__':
    import rostest
    rostest.unitrun('calibration_estimation', 'test_ChainBundler', TestChainBundler)
    rostest.unitrun('calibration_estimation', 'test_ChainSensor', TestChainSensor)
