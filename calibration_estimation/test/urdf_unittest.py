#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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

from calibration_estimation.urdf_params import UrdfParams
from calibration_estimation.cal_bag_helpers import *

raw_xml = 'robot.xml'
calibrated_xml = 'robot_calibrated.xml'

def loadSystem():
    robot_description = get_robot_description('/tmp/pr2_calibration/cal_measurements.bag')
    outfile = open(raw_xml, 'w')
    outfile.write(robot_description)
    outfile.close()
    config = yaml.load('''
sensors:
  chains: {}
  rectified_cams: {}
  tilting_lasers: {}
transforms: {}
checkerboards: {}
''')    
    return UrdfParams(robot_description, config)

class TestUrdfWriter(unittest.TestCase):
    def test_write(self):
        print ""
        params = loadSystem()
        outfile = open(calibrated_xml, 'w')
        outfile.write( params.urdf.to_xml() )
        outfile.close()

if __name__ == '__main__':
    import rostest
    rostest.unitrun('calibration_estimation', 'test_UrdfWriter', TestUrdfWriter, coverage_packages=['calibration_estimation.urdf_params'])
