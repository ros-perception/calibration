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
#       before_chain_Ts -- camera_chain -- after_chain_Ts -- camera
#      /
#   root
#      \
#       before_chain_Ts -- target_chain -- after_chain_Ts -- checkerboard

import numpy
from numpy import reshape, array, zeros, diag, matrix, real
import roslib; roslib.load_manifest('calibration_estimation')
import rospy
from calibration_estimation.full_chain import FullChainRobotParams
from sensor_msgs.msg import JointState

class ChainBundler:
    def __init__(self, valid_configs):
        self._valid_configs = valid_configs

    # Construct a CameraChainRobotParamsBlock for every 'valid config' that finds everything it needs in the current robot measurement
    def build_blocks(self, M_robot):
        sensors = []
        for cur_config in self._valid_configs:
            if cur_config["sensor_id"] == M_robot.chain_id and \
                cur_config["sensor_id"] in [ x.chain_id  for x in M_robot.M_chain ]:
                M_chain = [x for x in M_robot.M_chain if cur_config["sensor_id"] == x.chain_id][0]
                cur_sensor = ChainSensor(cur_config, M_chain, M_robot.target_id)
                sensors.append(cur_sensor)
            else:
                rospy.logdebug("  Didn't find block")
        return sensors

class ChainSensor:
    def __init__(self, config_dict, M_chain, target_id):

        self.sensor_type = "chain"
        self.sensor_id   = config_dict["sensor_id"]

        self._config_dict = config_dict
        self._M_chain = M_chain
        self._target_id = target_id

        self._full_chain = FullChainRobotParams(self.sensor_id, self.sensor_id+"_cb_link")

        self.terms_per_sample = 3

    def update_config(self, robot_params):
        self._full_chain.update_config(robot_params)
        self._checkerboard = robot_params.checkerboards[ self._target_id ]

    def compute_residual(self, target_pts):
        h_mat = self.compute_expected(target_pts)
        z_mat = self.get_measurement()
        assert(h_mat.shape == z_mat.shape)
        assert(h_mat.shape[0] == 4)
        r_mat = h_mat[0:3,:] - z_mat[0:3,:]
        r = array(reshape(r_mat.T, [-1,1]))[:,0]
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
            #print "k=%u" % k
            first = 3*k
            last = 3*k+3
            sub_cov = matrix(cov[first:last, first:last])
            sub_gamma_sqrt_full = matrix(scipy.linalg.sqrtm(sub_cov.I))
            sub_gamma_sqrt = real(sub_gamma_sqrt_full)
            assert(scipy.linalg.norm(sub_gamma_sqrt_full - sub_gamma_sqrt) < 1e-6)
            gamma[first:last, first:last] = sub_gamma_sqrt
        return gamma

    def compute_cov(self, target_pts):
        epsilon = 1e-8

        num_joints = len(self._M_chain.chain_state.position)
        Jt = zeros([num_joints, self.get_residual_length()])

        x = JointState()
        x.position = self._M_chain.chain_state.position[:]

        f0 = reshape(array(self._calc_fk_target_pts(x)[0:3,:].T), [-1])
        for i in range(num_joints):
            x.position = list(self._M_chain.chain_state.position[:])
            x.position[i] += epsilon
            fTest = reshape(array(self._calc_fk_target_pts(x)[0:3,:].T), [-1])
            Jt[i] = (fTest - f0)/epsilon
        cov_angles = [x*x for x in self._full_chain.calc_block._chain._cov_dict['joint_angles']]
        cov = matrix(Jt).T * matrix(diag(cov_angles)) * matrix(Jt)
        return cov

    def get_residual_length(self):
        pts = self._checkerboard.generate_points()
        N = pts.shape[1]
        return N*3

    def get_measurement(self):
        '''
        Returns a 4xN matrix with the locations of the checkerboard points in homogenous coords,
        as per the forward kinematics of the chain
        '''
        return self._calc_fk_target_pts(self._M_chain.chain_state)

    def _calc_fk_target_pts(self, chain_state):
        # Get the target's model points in the frame of the tip of the target chain
        target_pts_tip = self._checkerboard.generate_points()

        # Target pose in root frame
        target_pose_root = self._full_chain.calc_block.fk(chain_state)

        # Transform points into the root frame
        target_pts_root = target_pose_root * target_pts_tip

        return target_pts_root

    def compute_expected(self, target_pts):
        return target_pts

    # Build a dictionary that defines which parameters will in fact affect this measurement
    def build_sparsity_dict(self):
        sparsity = self._full_chain.build_sparsity_dict()
        sparsity['checkerboards'] = {}
        sparsity['checkerboards'][self._target_id] = { 'spacing_x': 1, 'spacing_y': 1 }
        return sparsity

