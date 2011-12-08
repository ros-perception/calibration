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
import time
import numpy
import yaml

from calibration_estimation.joint_chain import JointChain
from calibration_estimation.single_transform import SingleTransform
from calibration_estimation.urdf_params import UrdfParams
from sensor_msgs.msg import JointState

class LoadJointChain(unittest.TestCase):
    def setUp(self):
        print ""
        config = '''
root: x
tip: y
joints: [j1, j2, j3]
active_joints: [j1, j2, j3]
axis: [6, 6, 6]
gearing: [1, 1, 1]
cov:
  joint_angles: [1, 1, 1]
'''
        config_dict = yaml.load(config)
        config_dict['transforms'] = { 'j1': SingleTransform([1, 0, 0, 0, 0, 0]),
                                      'j2': SingleTransform([1, 0, 0, 0, 0, 0]),
                                      'j3': SingleTransform([1, 0, 2, 0, 0, 0]) }

        self.chain = JointChain(config_dict)

class TestJointChain(LoadJointChain):
    def test_init(self):
        pass

    def test_get_length(self):
        self.assertEqual(self.chain.get_length(), 3)

    def test_free(self):
        free_config = [ [ 1, 0, 0, 0, 0, 0 ],
                        [ 1, 0, 0, 0, 0, 0 ],
                        [ 1, 0, 0, 0, 0, 1 ] ]

        free_list = self.chain.calc_free({'transforms':free_config, 'axis': [6,6,6], 'gearing':[0,0,0]})
        self.assertEqual(free_list[0],  0)
        self.assertEqual(free_list[1],  0)
        self.assertEqual(free_list[2],  0)

    def test_deflate(self):
        param_vec = self.chain.deflate()
        self.assertEqual(param_vec[0,0], 1)
        self.assertEqual(param_vec[1,0], 1)
        self.assertEqual(param_vec[2,0], 1)

    def test_inflate(self):
        param_vec = numpy.reshape(numpy.matrix(numpy.arange(3)),(3,1))
        self.chain.inflate(param_vec)
        self.assertEqual(self.chain._gearing[0], 0)
        self.assertEqual(self.chain._gearing[2], 2)

    def test_to_params(self):
        param_vec = self.chain.deflate()
        param_vec[0,0] = 10
        config = self.chain.params_to_config(param_vec)
        self.assertAlmostEqual(config['gearing'][0], 10, 6)

    def test_fk_easy1(self):
        chain_state = JointState()
        chain_state.position = [0, 0, 0]
        eef = self.chain.fk(chain_state, 0)
        eef_expected = numpy.matrix( [[ 1, 0, 0, 1],
                                      [ 0, 1, 0, 0],
                                      [ 0, 0, 1, 0],
                                      [ 0, 0, 0, 1]] )
        self.assertAlmostEqual(numpy.linalg.norm(eef-eef_expected), 0.0, 6)

    def test_fk_easy2(self):
        chain_state = JointState()
        chain_state.position = [numpy.pi/2, 0, 0]
        eef = self.chain.fk(chain_state, 0)
        print eef
        eef_expected = numpy.matrix( [[ 0,-1, 0, 0],
                                      [ 1, 0, 0, 1],
                                      [ 0, 0, 1, 0],
                                      [ 0, 0, 0, 1]] )
        self.assertAlmostEqual(numpy.linalg.norm(eef-eef_expected), 0.0, 6)

    def test_fk_easy3(self):
        chain_state = JointState()
        chain_state.position = [numpy.pi/2, numpy.pi/2, 0]
        eef = self.chain.fk(chain_state, 1)
        print eef
        eef_expected = numpy.matrix( [[-1, 0, 0,-1],
                                      [ 0,-1, 0, 1],
                                      [ 0, 0, 1, 0],
                                      [ 0, 0, 0, 1]] )
        self.assertAlmostEqual(numpy.linalg.norm(eef-eef_expected), 0.0, 6)

    def test_fk_easy4(self):
        chain_state = JointState()
        chain_state.position = [0, 0, 0]
        eef = self.chain.fk(chain_state, -1)
        print eef
        eef_expected = numpy.matrix( [[ 1, 0, 0, 3],
                                      [ 0, 1, 0, 0],
                                      [ 0, 0, 1, 2],
                                      [ 0, 0, 0, 1]] )
        self.assertAlmostEqual(numpy.linalg.norm(eef-eef_expected), 0.0, 6)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('calibration_estimation', 'test_JointChain', TestJointChain, coverage_packages=['calibration_estimation.joint_chain'])
