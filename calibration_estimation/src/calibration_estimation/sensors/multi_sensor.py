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

# author: Vijay Pradeep

from calibration_estimation.sensors import tilting_laser_sensor, chain_sensor, camera_chain_sensor
from numpy import concatenate
from numpy import zeros, cumsum, matrix, array

def block_diag(m_list):
    '''
    Given a list of matricies.  Combine into a larger, block diagonal matrix. This really should
    exist in numpy
    '''
    # Must be square
    for m in m_list:
        assert(m.shape[0] == m.shape[1])
    m_sizes = [m.shape[0] for m in m_list ]
    end_ind   = list(cumsum(m_sizes))
    start_ind = [0] + end_ind[0:-1]
    result = zeros( [end_ind[-1], end_ind[-1] ] )
    for first, last, m in zip(start_ind, end_ind, m_list):
        result[first:last, first:last] = m
    return matrix(result)

class MultiSensor:
    '''
    Provides helper methods for dealing with all the sensor measurements
    generated from a single RobotMeasurement/CbPose pair
    '''
    def __init__(self, sensor_configs):
        self._sensor_configs = sensor_configs
        self.sensors = []
        self.checkerboard = "NONE"

    def sensors_from_message(self, msg):
        sensors = []

        sensor_type = 'tilting_lasers'
        if sensor_type in self._sensor_configs.keys():
            cur_bundler = tilting_laser_sensor.TiltingLaserBundler( self._sensor_configs[sensor_type] )
            cur_sensors = cur_bundler.build_blocks(msg)
            sensors.extend(cur_sensors)
        else:
            print "[%s] section doesn't exist. Skipping" % sensor_type

        sensor_type = 'chains'
        if sensor_type in self._sensor_configs.keys():
            cur_bundler = chain_sensor.ChainBundler( self._sensor_configs[sensor_type] )
            cur_sensors = cur_bundler.build_blocks(msg)
            sensors.extend(cur_sensors)
        else:
            print "[%s] section doesn't exist. Skipping" % sensor_type

        sensor_type = 'rectified_cams'
        if sensor_type in self._sensor_configs.keys():
            cur_bundler = camera_chain_sensor.CameraChainBundler( self._sensor_configs[sensor_type] )
            cur_sensors = cur_bundler.build_blocks(msg)
            sensors.extend(cur_sensors)
        else:
            print "[%s] section doesn't exist. Skipping" % sensor_type

        # Store the sensor list internally
        self.sensors = sensors
        self.checkerboard = msg.target_id

    def update_config(self, robot_params):
        for sensor in self.sensors:
            sensor.update_config(robot_params)

    def compute_residual(self, target_pts):
        r_list = [sensor.compute_residual(target_pts) for sensor in self.sensors]
        if len(r_list) == 0:
            return array([])

        r = concatenate(r_list,0)
        return r

    def compute_residual_scaled(self, target_pts):
        r_list = [sensor.compute_residual_scaled(target_pts) for sensor in self.sensors]
        if len(r_list) == 0:
            return array([])

        r = concatenate(r_list,0)
        return r

    def compute_marginal_gamma_sqrt(self, target_pts):
        gamma_sqrt_list = [sensor.compute_marginal_gamma_sqrt(target_pts) for sensor in self.sensors]
        if len(gamma_sqrt_list) == 0:
            return matrix([])
        gamma_sqrt = block_diag(gamma_sqrt_list)
        return gamma_sqrt

    def get_residual_length(self):
        return sum([sensor.get_residual_length() for sensor in self.sensors])

