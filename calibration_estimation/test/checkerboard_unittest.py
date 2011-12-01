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

from calibration_estimation.checkerboard import Checkerboard
from numpy import *

class TestCheckerboard(unittest.TestCase):
    def test_init(self):
        cb = Checkerboard( {"corners_x": 2,
                            "corners_y": 3,
                            "spacing_x": 10,
                            "spacing_y": 20 } )

        self.assertEqual(cb._corners_x, 2)
        self.assertEqual(cb._corners_y, 3)
        self.assertEqual(cb._spacing_x, 10)
        self.assertEqual(cb._spacing_y, 20)

    def test_params_to_config(self):
        cb = Checkerboard( {"corners_x": 2,
                            "corners_y": 3,
                            "spacing_x": 10,
                            "spacing_y": 20 } )
        config = cb.params_to_config( matrix([5,6], float).T )
        self.assertEquals( config["corners_x"], 2)
        self.assertEquals( config["corners_y"], 3)
        self.assertAlmostEquals( config["spacing_x"], 5, 6)
        self.assertAlmostEquals( config["spacing_y"], 6, 6)

    def test_deflate(self):
        cb = Checkerboard()
        param_vec = matrix([10,20], float).T
        cb.inflate( param_vec )
        result = cb.deflate()
        print ""
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result - param_vec), 0.0, 6)

    def test_generate_points(self):
        cb = Checkerboard({"corners_x":  2,
                            "corners_y": 3,
                            "spacing_x": 10,
                            "spacing_y": 20 })
        result = cb.generate_points()
        expected = matrix( [ [ 0, 10,   0, 10,  0, 10],
                             [ 0,  0,  20, 20, 40, 40],
                             [ 0,  0,   0,  0,  0,  0],
                             [ 1,  1,   1,  1,  1,  1] ], float)
        print ""
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result - expected), 0.0, 6)

    def test_get_length(self):
        cb = Checkerboard()
        self.assertEqual(cb.get_length(), 2)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('calibration_estimation', 'test_Checkerboard', TestCheckerboard, coverage_packages=['calibration_estimation.checkerboard'])
