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

from calibration_estimation.tilting_laser import TiltingLaser
from calibration_estimation.urdf_params import UrdfParams
from numpy import *

def loadSystem1():
    urdf = '''
<robot>
  <link name="base_link"/>
  <joint name="j0" type="fixed">
    <origin xyz="0 0 10" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="j0_link"/>
  </joint>
  <link name="j0_link"/>
  <joint name="j1" type="revolute">
    <axis xyz="0 0 1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="j0_link"/>
    <child link="j1_link"/>
  </joint>
  <link name="j1_link"/>
  <joint name="j2" type="fixed">
    <origin xyz="20 0 0" rpy="0 0 0"/>
    <parent link="j1_link"/>
    <child link="j2_link"/>
  </joint>
  <link name="j2_link"/>
</robot>
'''
    config = yaml.load('''
sensors:
  chains: {}
  rectified_cams: {}
  tilting_lasers:
    test_laser:
      joint: j1
      frame_id: j2_link
      gearing: 1.0
      cov:
        bearing: 0.0005
        range: 0.005
        tilt: 0.0005
transforms: {}
checkerboards: {}
''')
        
    return UrdfParams(urdf, config)

class TestTiltingLaser(unittest.TestCase):

    def test_project_point_easy_1(self):
        params = loadSystem1()
        tl = params.tilting_lasers["test_laser"]
        tl.update_config(params)
        result = tl.project_point_to_3D([0, 0, 0])
        expected = numpy.matrix( [20, 0, 10, 1] ).T
        print ""
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result - expected), 0.0, 6)

    def test_project_point_easy_2(self):
        params = loadSystem1()
        tl = params.tilting_lasers["test_laser"]
        tl.update_config(params)
        result = tl.project_point_to_3D([0, pi/2, 1])
        expected = numpy.matrix( [20, 1, 10, 1] ).T
        print ""
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result - expected), 0.0, 6)

    def test_project_point_easy_3(self):
        params = loadSystem1()
        tl = params.tilting_lasers["test_laser"]
        tl.update_config(params)
        result = tl.project_point_to_3D([pi/2, 0, 0])
        expected = numpy.matrix( [0 , 0,-10, 1] ).T
        print ""
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result - expected), 0.0, 6)

    def test_project_point_easy_4(self):
        params = loadSystem1()
        tl = params.tilting_lasers["test_laser"]
        tl.update_config(params)
        result = tl.project_point_to_3D([pi/2, -pi/2, 15])
        expected = numpy.matrix( [0 ,-15, -10, 1] ).T
        print ""
        print result
        self.assertAlmostEqual(numpy.linalg.norm(result - expected), 0.0, 6)

    def test_project(self):
        params = loadSystem1()
        tl = params.tilting_lasers["test_laser"]
        tl.update_config(params)

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
        params = loadSystem1()
        tl = params.tilting_lasers["test_laser"]
        self.assertEqual(tl.get_length(), 1)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('calibration_estimation', 'test_TiltingLaser', TestTiltingLaser, coverage_packages=['calibration_estimation.tilting_laser'])

