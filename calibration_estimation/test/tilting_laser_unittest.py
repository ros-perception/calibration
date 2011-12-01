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

from calibration_estimation.tilting_laser import TiltingLaser
from numpy import *

class TestTiltingLaser(unittest.TestCase):

    def test_free(self):
        tl = TiltingLaser( {"before_joint":[ 0, 0, 10, 0,    0, pi/2],
                            "after_joint": [20, 0,  0, 0, pi/2,    0],
                            "gearing":1} )
        free_list = tl.calc_free( {"before_joint":[ 0, 0, 1, 0, 0, 0 ],
                                   "after_joint": [ 1, 0, 0, 0, 0, 0 ],
                                   "gearing":0} )

        self.assertEqual(free_list[0], False)
        self.assertEqual(free_list[1], False)
        self.assertEqual(free_list[2], True)
        self.assertEqual(free_list[6], True)
        self.assertEqual(free_list[11],False)
        self.assertEqual(free_list[12],False)

    def test_inflate(self):
        tl = TiltingLaser( {"before_joint":[ 0, 0, 10, 0,    0, pi/2],
                            "after_joint": [20, 0,  0, 0, pi/2,    0],
                            "gearing":1 } )
        expected_before = numpy.matrix( [[ 0,-1, 0,  0],
                                         [ 1, 0, 0,  0],
                                         [ 0, 0, 1, 10],
                                         [ 0, 0, 0,  1]], float )

        expected_after  = numpy.matrix( [[ 0, 0, 1, 20],
                                         [ 0, 1, 0,  0],
                                         [-1, 0, 0,  0],
                                         [ 0, 0, 0,  1]], float )

        print ""
        print "Before Joint"
        print tl._before_joint.transform
        print "After Joint"
        print tl._after_joint.transform

        self.assertAlmostEqual(numpy.linalg.norm(tl._before_joint.transform - expected_before), 0.0, 6)
        self.assertAlmostEqual(numpy.linalg.norm(tl._after_joint.transform  - expected_after),  0.0, 6)

    def test_deflate(self):
        tl = TiltingLaser()
        p = reshape ( matrix( [ 1,  2,  3,  4,  5,  6,
                                7,  8,  9, 10, 11, 12,
                                13 ] ), (-1,1))
        tl.inflate(p)
        result = tl.deflate()
        #import code; code.interact(local=locals())
        self.assertAlmostEqual(numpy.linalg.norm(p - result), 0.0, 6)

    def test_params_to_config(self):
        tl = TiltingLaser()
        p = reshape ( matrix( [  1,  2,  3,  4,  5,  6,
                                 7,  8,  9, 10, 11, 12, 13 ] ), (-1,1))
        config = tl.params_to_config(p)
        self.assertAlmostEqual(config["before_joint"][0], 1, 6)
        self.assertAlmostEqual(config["after_joint"][2], 9, 6)

    def test_project_point_easy_1(self):
        tl = TiltingLaser( {"before_joint": [  0, 0, 10, 0, 0, 0],
                            "after_joint" : [ 20, 0,  0, 0, 0, 0],
                            "gearing":1 } )
        result = tl.project_point_to_3D([0, 0, 0])
        expected = numpy.matrix( [20, 0, 10, 1] ).T
        print ""
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result - expected), 0.0, 6)

    def test_project_point_easy_2(self):
        tl = TiltingLaser( {"before_joint": [  0, 0, 10, 0, 0, 0],
                            "after_joint" : [ 20, 0,  0, 0, 0, 0],
                            "gearing":1 } )
        result = tl.project_point_to_3D([0, pi/2, 1])
        expected = numpy.matrix( [20, 1, 10, 1] ).T
        print ""
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result - expected), 0.0, 6)

    def test_project_point_easy_3(self):
        tl = TiltingLaser( {"before_joint": [  0,  0, 10, 0, 0, 0],
                            "after_joint" : [ 20,  0,  0, 0, 0, 0],
                            "gearing":1 } )
        result = tl.project_point_to_3D([pi/2, 0, 0])
        expected = numpy.matrix( [0 , 0,-10, 1] ).T
        print ""
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result - expected), 0.0, 6)


    def test_project_point_easy_4(self):
        tl = TiltingLaser( {"before_joint": [  0,  0, 10, 0, 0, 0],
                            "after_joint" : [ 20,  0,  0, 0, 0, 0],
                            "gearing":1 } )
        result = tl.project_point_to_3D([pi/2, -pi/2, 15])
        expected = numpy.matrix( [0 ,-15, -10, 1] ).T
        print ""
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result - expected), 0.0, 6)


    def test_project(self):
        tl = TiltingLaser( {"before_joint": [  0,  0, 10, 0, 0, 0],
                            "after_joint" : [ 20,  0,  0, 0, 0, 0],
                            "gearing":1 } )

        result = tl.project_to_3D( [ [0,    0, 0],
                                     [0,    0, 1],
                                     [pi/2, 0, 0],
                                     [pi/2, pi/2,1] ] )

        expected = numpy.matrix( [ [ 20, 21,   0,   0 ],
                                   [  0,  0,   0,   1 ],
                                   [ 10, 10, -10, -10 ],
                                   [  1,  1,   1,   1 ] ] )

        print
        print result

        self.assertAlmostEqual(numpy.linalg.norm(result - expected), 0.0, 6)

    def test_get_length(self):
        tl = TiltingLaser()
        self.assertEqual(tl.get_length(), 13)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('calibration_estimation', 'test_TiltingLaser', TestTiltingLaser, coverage_packages=['calibration_estimation.tilting_laser'])

