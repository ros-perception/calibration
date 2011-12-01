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

from calibration_estimation.single_transform import SingleTransform
from numpy import *

class TestSingleTransform(unittest.TestCase):

    def test_free(self):

        st = SingleTransform([0, 0, 0, 0, 0, 0])
        free_list = st.calc_free( [0, 0, 1, 1, 0, 0] )

        self.assertEqual(free_list[0], False)
        self.assertEqual(free_list[1], False)
        self.assertEqual(free_list[2], True)
        self.assertEqual(free_list[3], True)
        self.assertEqual(free_list[4], False)
        self.assertEqual(free_list[5], False)

    def test_inflate(self):
        st = SingleTransform([0, 0, 0, 0, 0, 0])
        st.inflate( reshape( matrix([1, 0, 0, 0, 0, 0], float), (-1,1) ))
        expected = numpy.matrix( [[ 1, 0, 0, 1],
                                  [ 0, 1, 0, 0],
                                  [ 0, 0, 1, 0],
                                  [ 0, 0, 0, 1]], float )

        print ""
        print st.transform

        self.assertAlmostEqual(numpy.linalg.norm(st.transform-expected), 0.0, 6)

    def test_deflate(self):
        st = SingleTransform([0, 0, 0, 0, 0, 0])
        p = reshape( matrix([1, 0, 0, 0, 0, 0], float), (-1,1) )
        st.inflate(p)
        result = st.deflate()
        self.assertAlmostEqual(numpy.linalg.norm(p-result), 0.0, 6)

    def test_params_to_config(self):
        st = SingleTransform()
        p = reshape( matrix([1, 0, 0, 0, 0, 0], float), (-1,1) )
        config = st.params_to_config(p)
        self.assertAlmostEqual( config[0], 1, 6)
        self.assertAlmostEqual( config[1], 0, 6)


    def test_easy_trans_1(self):
        st = SingleTransform([1, 2, 3, 0, 0, 0])
        expected = numpy.matrix( [[ 1, 0, 0, 1],
                                  [ 0, 1, 0, 2],
                                  [ 0, 0, 1, 3],
                                  [ 0, 0, 0, 1]], float )

        print ""
        print st.transform

        self.assertAlmostEqual(numpy.linalg.norm(st.transform-expected), 0.0, 6)

    def test_easy_rot_1(self):
        st = SingleTransform([0, 0, 0, 0, 0, pi/2])
        expected = numpy.matrix( [[ 0,-1, 0, 0],
                                  [ 1, 0, 0, 0],
                                  [ 0, 0, 1, 0],
                                  [ 0, 0, 0, 1]], float )
        print ""
        print st.transform

        self.assertAlmostEqual(numpy.linalg.norm(st.transform-expected), 0.0, 6)

    def test_easy_rot_2(self):
        st = SingleTransform([0, 0, 0, 0, pi/2, 0])
        expected = numpy.matrix( [[ 0, 0, 1, 0],
                                  [ 0, 1, 0, 0],
                                  [-1, 0, 0, 0],
                                  [ 0, 0, 0, 1]], float )
        print ""
        print st.transform

        self.assertAlmostEqual(numpy.linalg.norm(st.transform-expected), 0.0, 6)

    def test_easy_rot_3(self):
        st = SingleTransform([0, 0, 0, pi/2, 0, 0])
        expected = numpy.matrix( [[ 1, 0, 0, 0],
                                  [ 0, 0,-1, 0],
                                  [ 0, 1, 0, 0],
                                  [ 0, 0, 0, 1]], float )
        print ""
        print st.transform

        self.assertAlmostEqual(numpy.linalg.norm(st.transform-expected), 0.0, 6)

    def test_length(self):
        st = SingleTransform([0, 0, 0, 0, 0, 0])
        self.assertEqual(st.get_length(), 6)

    def test_hard(self):
        sample_nums = range(1,6)
        params_filenames    = ["test/data/single_transform_data/params_%02u.txt" % n for n in sample_nums]
        transform_filenames = ["test/data/single_transform_data/transform_%02u.txt" % n for n in sample_nums]

        for params_filename, transform_filename in zip(params_filenames, transform_filenames):
            f_params = open(params_filename)
            f_transforms = open(transform_filename)
            param_elems     = [float(x) for x in f_params.read().split()]
            transform_elems = [float(x) for x in f_transforms.read().split()]

            st = SingleTransform(param_elems)
            expected_result = numpy.matrix(transform_elems)
            expected_result.shape = 4,4

            self.assertAlmostEqual(numpy.linalg.norm(st.transform-expected_result), 0.0, 4, "Failed on %s" % params_filename)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2_calibration_estimation', 'test_SingleTransform', TestSingleTransform, coverage_packages=['pr2_calibration_estimation.single_transform'])

