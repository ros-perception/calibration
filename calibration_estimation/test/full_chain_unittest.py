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
from sensor_msgs.msg import JointState
from calibration_estimation.full_chain import FullChainCalcBlock
from calibration_estimation.single_transform import SingleTransform
from calibration_estimation.joint_chain import JointChain

from numpy import *
import numpy

def loadSystem1():
    calc_block = FullChainCalcBlock()
    before_chain_Ts = [SingleTransform([10, 0, 0, 0, 0, 0])]
    chain = JointChain( {'transforms':[ [1, 0, 0, 0, 0, 0] ],
                         'gearing':[1],
                         'axis':[6],
                         'cov':{'joint_angles':[1]}} )
    link_num = -1
    after_chain_Ts = [SingleTransform([ 0, 0, 20, 0, 0, 0])]

    calc_block.update_config(before_chain_Ts, chain, link_num, after_chain_Ts)
    return calc_block


class TestFullChainCalcBlock(unittest.TestCase):
    def test_fk_1(self):
        print ""
        calc_block = loadSystem1()
        chain_state = JointState(position=[0])
        result = calc_block.fk(chain_state)
        expected = numpy.matrix( [[ 1, 0, 0,11],
                                  [ 0, 1, 0, 0],
                                  [ 0, 0, 1,20],
                                  [ 0, 0, 0, 1]], float )
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)

    def test_fk_2(self):
        print ""
        calc_block = loadSystem1()
        chain_state = JointState(position=[pi/2])
        result = calc_block.fk(chain_state)
        expected = numpy.matrix( [[ 0,-1, 0,10],
                                  [ 1, 0, 0, 1],
                                  [ 0, 0, 1,20],
                                  [ 0, 0, 0, 1]], float )
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('calibration_estimation', 'test_FullChainCalcBlock',   TestFullChainCalcBlock,   coverage_packages=['calibration_estimation.full_chain'])
