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
from calibration_estimation.sensors.tilting_laser_sensor import TiltingLaserBundler, TiltingLaserSensor
from calibration_msgs.msg import *
from sensor_msgs.msg import JointState, CameraInfo
from calibration_estimation.urdf_params import UrdfParams

from calibration_estimation.tilting_laser import TiltingLaser

import numpy
from numpy import *

def loadConfigList():

    config_yaml = '''
- laser_id:     laser1
  sensor_id:    laser1
'''
    config_dict = yaml.load(config_yaml)
    return config_dict

class TestTiltingLaserBundler(unittest.TestCase):
    def test_basic(self):
        config_list = loadConfigList()

        bundler = TiltingLaserBundler(config_list)

        M_robot = RobotMeasurement()
        M_robot.M_laser.append( LaserMeasurement(laser_id="laser1"))\

        blocks = bundler.build_blocks(M_robot)

        self.assertEqual( len(blocks), 1)
        self.assertEqual( blocks[0]._M_laser.laser_id, "laser1")

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
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="j0_link"/>
    <child link="j1_link"/>
  </joint>
  <link name="j1_link"/>
  <joint name="j2" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
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
    laserA:
      sensor_id: laserA
      joint: j1
      frame_id: j2_link
      gearing: 1
      cov:
        bearing: 1
        range: 1
        tilt: 1
transforms: {}
checkerboards: {}
''')
    
    return config["sensors"], UrdfParams(urdf, config)


class TestTiltingLaser(unittest.TestCase):

    def test_cov(self):
        print ""
        config, robot_params = loadSystem()

        joint_points = [ JointState(position=[0,0,1]),
                         JointState(position=[pi/2,0,2]) ]

        sensor = TiltingLaserSensor(config["tilting_lasers"]["laserA"], LaserMeasurement(laser_id="laserA",
                                                             joint_points=joint_points))

        sensor.update_config(robot_params)

        cov = sensor.compute_cov(None)

        print "Cov:"
        print cov

        self.assertAlmostEqual(cov[0,0], 1.0, 6)
        self.assertAlmostEqual(cov[1,1], 1.0, 6)
        self.assertAlmostEqual(cov[2,2], 1.0, 6)
        self.assertAlmostEqual(cov[3,3], 4.0, 6)
        self.assertAlmostEqual(cov[4,4], 4.0, 6)
        self.assertAlmostEqual(cov[5,5], 1.0, 6)

    def test_gamma(self):
        print ""
        config, robot_params = loadSystem()

        joint_points = [ JointState(position=[0,0,1]),
                         JointState(position=[pi/2,0,2]) ]

        sensor = TiltingLaserSensor(config["tilting_lasers"]["laserA"], LaserMeasurement(laser_id="laserA",
                                                             joint_points=joint_points))

        sensor.update_config(robot_params)
        gamma = sensor.compute_marginal_gamma_sqrt(None)

        print "Gamma:"
        print gamma
        self.assertAlmostEqual(gamma[0,0], 1.0, 6)
        self.assertAlmostEqual(gamma[1,1], 1.0, 6)
        self.assertAlmostEqual(gamma[2,2], 1.0, 6)
        self.assertAlmostEqual(gamma[3,3], 0.5, 6)
        self.assertAlmostEqual(gamma[4,4], 0.5, 6)
        self.assertAlmostEqual(gamma[5,5], 1.0, 6)

    def test_tilting_laser_1(self):
        print ""
        config, robot_params = loadSystem()

        joint_points = [ JointState(position=[0,0,0]),
                         JointState(position=[0,pi/2,1]),
                         JointState(position=[pi/2,0,1]) ]

        sensor = TiltingLaserSensor(config["tilting_lasers"]["laserA"], LaserMeasurement(laser_id="laserA",
                                                            joint_points=joint_points))

        sensor.update_config(robot_params)

        target_pts = matrix( [ [ 0,  0,  0 ],
                               [ 0,  1,  0 ],
                               [ 0,  0, -1 ],
                               [ 1,  1,  1 ] ] )

        h = sensor.compute_expected(target_pts)
        z = sensor.get_measurement()
        r = sensor.compute_residual(target_pts)

        self.assertAlmostEqual(numpy.linalg.norm(h-target_pts), 0.0, 6)
        self.assertAlmostEqual(numpy.linalg.norm(z-target_pts), 0.0, 6)
        self.assertAlmostEqual(numpy.linalg.norm(r), 0.0, 6)

        # Test Sparsity
        sparsity = sensor.build_sparsity_dict()
        self.assertEqual(sparsity['transforms']['j0'], [1,1,1,1,1,1])
        self.assertEqual(sparsity['transforms']['j1'], [1,1,1,1,1,1])
        self.assertEqual(sparsity['transforms']['j2'], [1,1,1,1,1,1])
        self.assertEqual(sparsity['tilting_lasers']['laserA']['gearing'], 1)


if __name__ == '__main__':
    import rostest
    rostest.unitrun('calibration_estimation', 'test_TiltingLaserBundler',   TestTiltingLaserBundler,   coverage_packages=['calibration_estimation.sensors.tilting_laser_sensor'])
    rostest.unitrun('calibration_estimation', 'test_TiltingLaser', TestTiltingLaser, coverage_packages=['calibration_estimation.sensors.tilting_laser_sensor'])
