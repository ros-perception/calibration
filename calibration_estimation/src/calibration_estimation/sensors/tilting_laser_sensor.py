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

# Error block for a monocular camera plus tilting laser.  The
# camera is attached to a chain, but also has a 6 dof transform
# in between the tilting laser is attached to the
#
#       checkerboard
#      /
#   root
#      \
#       tilting_laser

from numpy import reshape, array, zeros, matrix, diag, real

import roslib; roslib.load_manifest('calibration_estimation')
import rospy
import numpy

class TiltingLaserBundler:
    def __init__(self, valid_configs):
        self._valid_configs = valid_configs

    # Construct a tilitng laser for each laser pair that matches one of the configs
    def build_blocks(self, M_robot):
        sensors = []
        for cur_config in self._valid_configs:
            if cur_config["sensor_id"] in [ x.laser_id  for x in M_robot.M_laser ] :
                M_laser = [x for x in M_robot.M_laser if cur_config["sensor_id"] == x.laser_id][0]
                cur_sensor = TiltingLaserSensor(cur_config, M_laser)
                sensors.append(cur_sensor)
            else:
                rospy.logdebug("  Didn't find block")
        return sensors

class TiltingLaserSensor:
    def __init__(self, config_dict, M_laser):
        self.sensor_type = "laser"
        self.sensor_id = config_dict["sensor_id"]

        self._config_dict = config_dict
        self._M_laser = M_laser
        self.terms_per_sample = 3

    def update_config(self, robot_params):
        self._tilting_laser = robot_params.tilting_lasers[ self.sensor_id ]
        self._tilting_laser.update_config(robot_params)

    def compute_residual(self, target_pts):
        z_mat = self.get_measurement()
        h_mat = self.compute_expected(target_pts)
        assert(z_mat.shape[0] == 4)
        assert(h_mat.shape[0] == 4)
        assert(z_mat.shape[1] == z_mat.shape[1])
        r = array(reshape(h_mat[0:3,:].T - z_mat[0:3,:].T, [-1,1]))[:,0]
        return r

    def compute_residual_scaled(self, target_pts):
        r = self.compute_residual(target_pts)
        gamma_sqrt = self.compute_marginal_gamma_sqrt(target_pts)
        r_scaled = gamma_sqrt * matrix(r).T
        return array(r_scaled.T)[0]

    def compute_marginal_gamma_sqrt(self, target_pts):
        import scipy.linalg
        # ----- Populate Here -----
        cov = self.compute_cov(target_pts)
        gamma = matrix(zeros(cov.shape))
        num_pts = self.get_residual_length()/3

        for k in range(num_pts):
            first = 3*k
            last = 3*k+3
            sub_cov = matrix(cov[first:last, first:last])
            sub_gamma_sqrt_full = matrix(scipy.linalg.sqrtm(sub_cov.I))
            sub_gamma_sqrt = real(sub_gamma_sqrt_full)
            assert(scipy.linalg.norm(sub_gamma_sqrt_full - sub_gamma_sqrt) < 1e-6)
            gamma[first:last, first:last] = sub_gamma_sqrt
        return gamma

    def get_residual_length(self):
        N = len(self._M_laser.joint_points)
        return N*3

    # Get the laser measurement.
    # Returns a 3xN matrix of points in cartesian space
    def get_measurement(self):
        # Get the laser points in a 4xN homogenous matrix
        laser_pts_root = self._tilting_laser.project_to_3D([x.position for x in self._M_laser.joint_points])
        return laser_pts_root

    def compute_cov(self, target_pts):
        epsilon = 1e-8

        # Ok, this is coded kind of weird. Jt should be [3*N, len(r)]. Currently, it is
        # [3, len(r)], which means we are assuming that laser noise is correlated across
        # multiple points. This is wrong.  However, we accidentally fix this when computing
        # Gamma, so it doesn't show up in the final solution.
        Jt = zeros([3, self.get_residual_length()])

        import copy
        x = [copy.copy(x.position) for x in self._M_laser.joint_points]

        f0 = reshape(array(self._tilting_laser.project_to_3D(x)[0:3,:].T), [-1])
        for i in range(3):
            x = [ [y for y in x.position] for x in self._M_laser.joint_points]
            for cur_pt in x:
                cur_pt[i] += epsilon
            fTest = reshape(array(self._tilting_laser.project_to_3D(x)[0:3,:].T), [-1])
            Jt[i] = (fTest - f0)/epsilon

        num_pts = len(x)
        std_dev_sensor = [self._tilting_laser._cov_dict['tilt'],
                          self._tilting_laser._cov_dict['bearing'],
                          self._tilting_laser._cov_dict['range']]
        cov_sensor = [x*x for x in std_dev_sensor]

        cov = matrix(Jt).T * matrix(diag(cov_sensor)) * matrix(Jt)
        return cov

    # Returns the pixel coordinates of the laser points after being projected into the camera
    # Returns a 4xN matrix of the target points
    def compute_expected(self, target_pts):
        return target_pts

    # Build a dictionary that defines which parameters will in fact affect this measurement
    def build_sparsity_dict(self):
        sparsity = dict()

        sparsity['tilting_lasers'] = {}
        sparsity['tilting_lasers'][self.sensor_id] = {'gearing':1}
        sparsity['transforms'] = {}
        for cur_transform in ( self._tilting_laser._before_chain_Ts + \
                               self._tilting_laser._after_chain_Ts ): 
            sparsity['transforms'][cur_transform._name] = [1, 1, 1, 1, 1, 1]
        sparsity['transforms'][self._tilting_laser._config['joint']] = [1, 1, 1, 1, 1, 1]

        return sparsity

