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

import numpy
from numpy import matrix, vsplit, sin, cos, reshape
import rospy
from calibration_estimation.single_transform import SingleTransform

# Primitive used to model a tilting laser platform.

param_names = ['gearing']

class TiltingLaser:

    def __init__(self, config ):
        rospy.logdebug("Initializing tilting laser")
        self._config = config
        self._cov_dict = config['cov']

        param_vec = self.dict_to_params(config)
        self.inflate(param_vec)

    def update_config(self, robot_params):
        parent = robot_params.urdf.joints[self._config['joint']].parent
        child = robot_params.urdf.joints[self._config['joint']].child
        before_chain = robot_params.urdf.get_chain(robot_params.base_link, parent, links=False)
        before_chain.append(self._config['joint'])
        after_chain = robot_params.urdf.get_chain(child, self._config['frame_id'], links=False)
        self._before_chain_Ts = [robot_params.transforms[transform_name] for transform_name in before_chain]
        self._after_chain_Ts  = [robot_params.transforms[transform_name] for transform_name in after_chain]

    def dict_to_params(self, config):
        param_vec = matrix(numpy.zeros((1,1), float))
        param_vec[0,0] = config['gearing']
        return param_vec

    def params_to_config(self, param_vec):
        return {'joint'     : self._config['joint'],
                'frame_id'  : self._config['frame_id'],
                "gearing"   : float(param_vec[0,0]),
                "cov"       : self._cov_dict }

    def calc_free(self, free_config):
        return [free_config['gearing'] == 1]

    # Convert column vector of params into config
    def inflate(self, param_vec):
        self._gearing = param_vec[0,0]

    # Return column vector of config
    def deflate(self):
        param_vec = matrix(numpy.zeros((1,1), float))
        param_vec[0,0] = self._gearing
        return param_vec

    # Returns # of params needed for inflation & deflation
    def get_length(self):
        return len(param_names)

    def compute_pose(self, joint_pos):
        pose = matrix(numpy.eye(4))
        for before_chain_T in self._before_chain_Ts:
            pose = pose * before_chain_T.transform
        pose = pose * SingleTransform([0, 0, 0, 0, joint_pos[0]*self._gearing, 0]).transform # TODO: remove assumption of Y axis
        for after_chain_T in self._after_chain_Ts:
            pose = pose * after_chain_T.transform
        return pose

    # Given a single set of joint positions, project into 3D
    # joint_pos - a single list that looks like [tilting_joint, pointing_angle, range]
    # returns - 4x1 numpy matrix, with the resulting projected point (in homogenous coords)
    def project_point_to_3D(self, joint_pos):
        p = joint_pos[1] # Pointing Angle
        r = joint_pos[2] # Range
        ray = reshape(matrix([cos(p)*r, sin(p)*r, 0, 1]), (-1,1))

        result = self.compute_pose(joint_pos) * ray
        return result

    # Take sets of joint positions, and project them into 3D
    # joint_positions - should look like the following
    #   - [tilting_joint, pointing_angle, range]   # Point 1
    #   - [tilting_joint, pointing_angle, range]   # Point 2
    #   -                     :
    #   - [tilting_joint, pointing_angle, range]   # Point N
    # Return - 4xN numpy matrix of 3D points
    def project_to_3D(self, joint_pos):
        result = numpy.concatenate( [self.project_point_to_3D(x) for x in joint_pos], 1 )
        return result

