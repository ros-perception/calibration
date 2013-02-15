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

# Define a camera sensor attached to a chain
#
#       before_chain_Ts -- camera_chain -- after_chain_Ts -- camera
#      /
#   root
#      \
#       checkerboard

import numpy
from numpy import matrix, reshape, array, zeros, diag, real

import roslib; roslib.load_manifest('calibration_estimation')
import rospy
from calibration_estimation.full_chain import FullChainRobotParams
from sensor_msgs.msg import JointState

class CameraChainBundler:
    """
    Tool used to generate a list of CameraChain sensors from a single calibration sample message
    """
    def __init__(self, valid_configs):
        self._valid_configs = valid_configs

    # Construct a CameraChainSensor for every camera chain sensor that exists in the given robot measurement
    def build_blocks(self, M_robot):
        """
        Input:
        - M_robot: A calibration sample of type calibration_msgs/RobotMeasurement
        Returns:
        - sensors: A list of CameraChainSensor objects that corresponds to all camera chain sensors found
                   in the calibration message that was passed in.
        """
        sensors = []
        for cur_config in self._valid_configs:
            if cur_config["sensor_id"] in [ x.camera_id for x in M_robot.M_cam ]:
                if "chain_id" in cur_config.keys() and cur_config["chain_id"] != None and cur_config["chain_id"] != "NONE":
                    if cur_config["chain_id"] in [ x.chain_id  for x in M_robot.M_chain ] :
                        M_cam   = M_robot.M_cam  [ [ x.camera_id for x in M_robot.M_cam   ].index(cur_config["sensor_id"])]
                        M_chain = M_robot.M_chain[ [ x.chain_id  for x in M_robot.M_chain ].index(cur_config["chain_id"]) ]
                    else:
                        continue
                else:
                    M_cam   = M_robot.M_cam  [ [ x.camera_id for x in M_robot.M_cam   ].index(cur_config["sensor_id"])]
                    M_chain = None
                    cur_config["chain_id"] = None
                cur_sensor = CameraChainSensor(cur_config, M_cam, M_chain)
                sensors.append(cur_sensor)
            else:
                rospy.logdebug("  Didn't find block")
        return sensors

class CameraChainSensor:
    def __init__(self, config_dict, M_cam, M_chain):
        """
        Generates a single sensor block for a single configuration
        Inputs:
        - config_dict: The configuration for this specific sensor. This looks exactly like
                       a single element from the valid_configs list passed into CameraChainBundler.__init__
        - M_cam: The camera measurement of type calibration_msgs/CameraMeasurement
        - M_chain: The chain measurement of type calibration_msgs/ChainMeasurement
        """
        self.sensor_type = "camera"
        self.sensor_id = config_dict["sensor_id"]

        self._config_dict = config_dict
        self._M_cam = M_cam
        self._M_chain = M_chain

        self._full_chain = FullChainRobotParams(config_dict["chain_id"], config_dict["frame_id"])

        self.terms_per_sample = 2
        if "baseline_rgbd" in config_dict.keys():
            self.terms_per_sample = 3

    def update_config(self, robot_params):
        """
        On each optimization cycle the set of system parameters that we're optimizing over will change. Thus,
        after each change in parameters we must call update_config to ensure that our calculated residual reflects
        the newest set of system parameters.
        """
        self._camera = robot_params.rectified_cams[ self.sensor_id ]
        if self._full_chain is not None:
            self._full_chain.update_config(robot_params)

    def compute_residual(self, target_pts):
        """
        Computes the measurement residual for the current set of system parameters and target points
        Input:
        - target_pts: 4XN matrix, storing features point locations in world cartesian homogenous coordinates.
        Output:
        - r: terms_per_sample*N long vector, storing pixel residuals for the target points in the form of
             [u1, v1, u2, v2, ..., uN, vN] or [u1, v1, u'1, u2....]
        """
        z_mat = self.get_measurement()
        h_mat = self.compute_expected(target_pts)
        r = array(reshape(h_mat - z_mat, [-1,1]))[:,0]
        return r

    def compute_residual_scaled(self, target_pts):
        """
        Computes the residual, and then scales it by sqrt(Gamma), where Gamma
        is the information matrix for this measurement (Cov^-1).
        """
        r = self.compute_residual(target_pts)
        gamma_sqrt = self.compute_marginal_gamma_sqrt(target_pts)
        r_scaled = gamma_sqrt * matrix(r).T
        return array(r_scaled.T)[0]

    def compute_marginal_gamma_sqrt(self, target_pts):
        """
        Calculates the square root of the information matrix for the measurement of the
        current set of system parameters at the passed in set of target points.
        """
        import scipy.linalg
        cov = self.compute_cov(target_pts)
        gamma = matrix(zeros(cov.shape))
        num_pts = self.get_residual_length()/self.terms_per_sample

        for k in range(num_pts):
            first = self.terms_per_sample*k
            last = self.terms_per_sample*k+self.terms_per_sample
            sub_cov = matrix(cov[first:last, first:last])
            sub_gamma_sqrt_full = matrix(scipy.linalg.sqrtm(sub_cov.I))
            sub_gamma_sqrt = real(sub_gamma_sqrt_full)
            assert(scipy.linalg.norm(sub_gamma_sqrt_full - sub_gamma_sqrt) < 1e-6)
            gamma[first:last, first:last] = sub_gamma_sqrt
        return gamma

    def get_residual_length(self):
        return self.terms_per_sample*len(self._M_cam.image_points)

    def get_measurement(self):
        """
        Get the target's pixel coordinates as measured by the actual sensor
        """
        if self.terms_per_sample == 2:
            return numpy.matrix([[pt.x, pt.y] for pt in self._M_cam.image_points])
        elif self.terms_per_sample == 3:
            return numpy.matrix([[pt.x, pt.y, self._camera.get_disparity(self._M_cam.cam_info.P, pt.z)] for pt in self._M_cam.image_points])

    def compute_expected(self, target_pts):
        """
        Compute the expected pixel coordinates for a set of target points.
        target_pts: 4xN matrix, storing feature points of the target, in homogeneous coords
        Returns: target points projected into pixel coordinates, in a Nx2 matrix
        """
        if self._M_chain is not None:
            return self._compute_expected(self._M_chain.chain_state, target_pts)
        else:
            return self._compute_expected(None, target_pts)
    def _compute_expected(self, chain_state, target_pts):
        """
        Compute what measurement we would expect to see, based on the current system parameters
        and the specified target point locations.

        Inputs:
        - chain_state: The joint angles of this sensor's chain of type sensor_msgs/JointState.
        - target_pts: 4XN matrix, storing features point locations in world cartesian homogenous coordinates.

        Returns: target points projected into pixel coordinates, in a Nx2 matrix
        """
        # Camera pose in root frame
        camera_pose_root = self._full_chain.calc_block.fk(chain_state)
        cam_frame_pts = camera_pose_root.I * target_pts
        # Do the camera projection
        pixel_pts = self._camera.project(self._M_cam.cam_info.P, cam_frame_pts)
        return pixel_pts.T

    def compute_expected_J(self, target_pts):
        """
        The output Jacobian J shows how moving target_pts in cartesian space affects
        the expected measurement in (u,v) camera coordinates.
        For n points in target_pts, J is a 2nx3n matrix
        Note: This doesn't seem to be used anywhere, except maybe in some drawing code
        """
        epsilon = 1e-8
        N = len(self._M_cam.image_points)
        Jt = zeros([N*3, N*self.terms_per_sample])
        for k in range(N):
            # Compute jacobian for point k
            sub_Jt = zeros([3,self.terms_per_sample])
            x = target_pts[:,k].copy()
            f0 = self.compute_expected(x)
            for i in [0,1,2]:
                x[i,0] += epsilon
                fTest = self.compute_expected(x)
                sub_Jt[i,:] = array((fTest - f0) / epsilon)
                x[i,0] -= epsilon
            Jt[k*3:(k+1)*3, k*self.terms_per_sample:(k+1)*self.terms_per_sample] = sub_Jt
        return Jt.T

    def compute_cov(self, target_pts):
        '''
        Computes the measurement covariance in pixel coordinates for the given
        set of target points (target_pts)
        Input:
         - target_pts: 4xN matrix, storing N feature points of the target, in homogeneous coords
        '''
        cam_cov = matrix(zeros([self.get_residual_length(), self.get_residual_length()]))

        # Convert StdDev into variance
        var_u = self._camera._cov_dict['u'] * self._camera._cov_dict['u']
        var_v = self._camera._cov_dict['v'] * self._camera._cov_dict['v']
        for k in range(cam_cov.shape[0]/self.terms_per_sample):
            cam_cov[self.terms_per_sample*k  , self.terms_per_sample*k]   = var_u
            cam_cov[self.terms_per_sample*k+1, self.terms_per_sample*k+1] = var_v
            if self.terms_per_sample == 3:
                cam_cov[self.terms_per_sample*k+2, self.terms_per_sample*k+2] = self._camera._cov_dict['x'] * self._camera._cov_dict['x']

        # Both chain and camera covariances are now in measurement space, so we can simply add them together
        if self._M_chain is not None:
            return self.get_chain_cov(target_pts) + cam_cov
        else:
            return cam_cov

    def get_chain_cov(self, target_pts, epsilon=1e-8):
        num_joints = len(self._M_chain.chain_state.position)
        Jt = zeros([num_joints, self.get_residual_length()])

        x = JointState()
        x.position = self._M_chain.chain_state.position[:]

        # Compute the Jacobian from the chain's joint angles to pixel residuals
        f0 = reshape(array(self._compute_expected(x, target_pts)), [-1])
        for i in range(num_joints):
            x.position = [cur_pos for cur_pos in self._M_chain.chain_state.position]
            x.position[i] += epsilon
            fTest = reshape(array(self._compute_expected(x, target_pts)), [-1])
            Jt[i] = (fTest - f0)/epsilon
        cov_angles = [x*x for x in self._full_chain.calc_block._chain._cov_dict['joint_angles']]

        # Transform the chain's covariance from joint angle space into pixel space using the just calculated jacobian
        return matrix(Jt).T * matrix(diag(cov_angles)) * matrix(Jt)
        
    def build_sparsity_dict(self):
        """
        Build a dictionary that defines which parameters will in fact affect this measurement.

        Returned dictionary should be of the following form:
          transforms:
            my_transformA: [1, 1, 1, 1, 1, 1]
            my_transformB: [1, 1, 1, 1, 1, 1]
          chains:
            my_chain:
              gearing: [1, 1, ---, 1]
          rectified_cams:
            my_cam:
              baseline_shift: 1
              f_shift: 1
              cx_shift: 1
              cy_shift: 1
        """
        sparsity = self._full_chain.build_sparsity_dict()
        sparsity['rectified_cams'] = {}
        sparsity['rectified_cams'][self.sensor_id] = dict( [(x,1) for x in self._camera.get_param_names()] )
        return sparsity

