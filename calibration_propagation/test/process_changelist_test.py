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

import pr2_calibration_propagation.process_changelist as process_changelist

class TestProcessChangelist(unittest.TestCase):
    # Remove characters
    def test_easy1(self):
        str_in = "012345"
        cl = [((0,1),'')]
        self.assertEqual(process_changelist.process_changelist(cl, str_in), "12345")

    # The nominal case
    def test_easy2(self):
        str_in = "012345"
        cl = [((1,3),'dog')]
        self.assertEqual(process_changelist.process_changelist(cl, str_in), "0dog345")

    # Replace the last character with something
    def test_append1(self):
        str_in = "012345"
        cl = [((5,6),'dog')]
        self.assertEqual(process_changelist.process_changelist(cl, str_in), "01234dog")

    # Add a change after all the existing elems
    def test_append2(self):
        str_in = "012345"
        cl = [((6,6),'dog')]
        self.assertEqual(process_changelist.process_changelist(cl, str_in), "012345dog")

    # Check when there are multiple changes
    def test_multi1(self):
        str_in = "012345"
        cl = [((1,2),'dog'),
              ((3,5),'cat')]
        self.assertEqual(process_changelist.process_changelist(cl, str_in), "0dog2cat5")

    # Check when there are no changes
    def test_empty1(self):
        str_in = "012345"
        self.assertEqual(process_changelist.process_changelist([], str_in), str_in)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2_calibration_propagation', 'test_process_changelist',   TestProcessChangelist,   coverage_packages=['pr2_calibration_propagation.process_changelist'])


