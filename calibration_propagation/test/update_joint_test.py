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

import roslib; roslib.load_manifest('pr2_calibration_propagation')

import sys
import unittest

import pr2_calibration_propagation.update_joint as update_joint

class TestUpdateJoint(unittest.TestCase):
    def test_easy1(self):
        str_in = '''<joint name="joint1"><calibration rising="123"/></joint>'''
        cl = update_joint.update_joint(str_in, 'joint1', ref_shift=1)
        self.assertEqual(len(cl), 1)
        self.assertEqual(cl[0][0], (42, 45))
        self.assertAlmostEqual(float(cl[0][1]), 124, 5)

class TestUpdateTransmission(unittest.TestCase):
    def test_easy1(self):
        str_in = '''<transmission name="trans1"><mechanicalReduction>2</mechanicalReduction></transmission>'''
        cl = update_joint.update_transmission(str_in, 'trans1', 3)
        self.assertEqual(len(cl), 1)
        self.assertEqual(cl[0][0], (49, 50))
        self.assertAlmostEqual(float(cl[0][1]), 6, 5)

    def test_easy2(self):
        str_in = '''<transmission name="trans2"></transmission><transmission name="trans1"><mechanicalReduction>2</mechanicalReduction></transmission>'''
        cl = update_joint.update_transmission(str_in, 'trans1', 3)
        self.assertEqual(len(cl), 1)
        self.assertEqual(cl[0][0], (92, 93))
        self.assertAlmostEqual(float(cl[0][1]), 6, 5)

class TestAttr(unittest.TestCase):
    def test_easy1(self):
        str_in = '''a="123" b="456"'''
        span = update_joint.find_attr_span(str_in, 'a')
        self.assertEqual(span, (3,6))

    def test_easy2(self):
        str_in = '''a = "123" b="456"'''
        span = update_joint.find_attr_span(str_in, 'a')
        self.assertEqual(span, (5,8))

    def test_easy3(self):
        str_in = '''a="123" b ="456"'''
        span = update_joint.find_attr_span(str_in, 'b')
        self.assertEqual(span, (12,15))

    def test_easy4(self):
        str_in = '''a="123" b= "456"'''
        span = update_joint.find_attr_span(str_in, 'b')
        self.assertEqual(span, (12,15))

    def test_not_found(self):
        str_in = '''a="123" b="456"'''
        span = update_joint.find_attr_span(str_in, 'c')
        self.assertEqual(span, None)

class TestSplitElem(unittest.TestCase):
    def test_easy1(self):
        str_in = '''<tag1/><tag2/><target name="blah"></target>'''
        span = update_joint.find_split_elem_span(str_in, 'target', 'blah')
        self.assertEqual(span, (14,43))

    def test_duplicate_elem(self):
        str_in = '''<tag1/><tag2/><target name="blah2"></target><target name="blah"></target>'''
        span = update_joint.find_split_elem_span(str_in, 'target', 'blah')
        self.assertEqual(span, (44,73))

class TestAtomicElem(unittest.TestCase):
    def test_easy1(self):
        str_in = '''<tag1/><target/><tag2></tag2>'''
        span = update_joint.find_atomic_elem_span(str_in, 'target')
        self.assertEqual(span, (7,16))

    def test_boundary(self):
        str_in = '''<target/>'''
        span = update_joint.find_atomic_elem_span(str_in, 'target')
        self.assertEqual(span, (0,9))

    def test_not_found(self):
        str_in = '''<target/>'''
        span = update_joint.find_atomic_elem_span(str_in, 'target2')
        self.assertEqual(span, None)

class TestSplitInternals(unittest.TestCase):
    def test_easy1(self):
        str_in = '''<tag1>123</tag1>'''
        span = update_joint.find_split_internals(str_in)
        self.assertEqual(span, (6,9))

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2_calibration_propagation', 'test_split_elem',   TestSplitElem,   coverage_packages=['pr2_calibration_propagation.update_joint'])
    rostest.unitrun('pr2_calibration_propagation', 'test_atomic_elem',  TestAtomicElem,  coverage_packages=['pr2_calibration_propagation.update_joint'])
    rostest.unitrun('pr2_calibration_propagation', 'test_attr',         TestAttr,        coverage_packages=['pr2_calibration_propagation.update_joint'])
    rostest.unitrun('pr2_calibration_propagation', 'test_update_joint', TestUpdateJoint, coverage_packages=['pr2_calibration_propagation.update_joint'])
    rostest.unitrun('pr2_calibration_propagation', 'test_split_internals', TestSplitInternals, coverage_packages=['pr2_calibration_propagation.update_joint'])
    rostest.unitrun('pr2_calibration_propagation', 'test_update_transmission', TestUpdateTransmission, coverage_packages=['pr2_calibration_propagation.update_joint'])
