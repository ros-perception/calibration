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

# Primitive used to model PR2's tilting laser platform. Consists of 2 fixed transforms.
# One before the joint, and one after. The joint axis is the x-axis after the first transform

class TiltingLaser:

    # Dictionary with two elems (Total of 13 elems)
    #  before_joint: [px py pz rx ry rx]
    #  after_joint:  [px py pz rx ry rx]
    def __init__(self, config = {"before_joint": [  0, 0, 0, 0, 0, 0],
                                 "after_joint" : [  0, 0, 0, 0, 0, 0],
                                 'gearing':1 } ):
        rospy.logdebug("Initializing tilting laser")

        self._before_joint = SingleTransform()
        self._after_joint  = SingleTransform()

        if 'cov' in config.keys():
            self._cov_dict = config['cov']
        else:
            self._cov_dict = {}

        param_vec = self.dict_to_params(config)
        self.inflate(param_vec)

    def dict_to_params(self, config):
        param_vec = reshape(matrix([ config["before_joint"], config["after_joint"] ], float), (-1,1))
        param_vec = numpy.concatenate([param_vec, matrix([config["gearing"]])])
        assert(param_vec.size == self.get_length())
        return param_vec

    def params_to_config(self, param_vec):
        assert(param_vec.shape == (13,1))
        return {"before_joint" : self._before_joint.params_to_config(param_vec[0:6, 0]),
                "after_joint"  : self._before_joint.params_to_config(param_vec[6:12,0]),
                "gearing"      : float(param_vec[12,0]),
                "cov"          : self._cov_dict}

    def calc_free(self, free_config):
        #import code; code.interact(local=locals())
        assert( 'before_joint' in free_config )
        assert( 'after_joint'  in free_config )
        assert( 'gearing' in free_config )

        # Flatten the config
        flat_config = free_config['before_joint'] + free_config['after_joint'] + [free_config['gearing']]
        assert( len(flat_config) == self.get_length())

        # Convert int list into bool list
        flat_free = [x == 1 for x in flat_config]

        return flat_free

    # Convert column vector of params into config
    def inflate(self, param_vec):
        self._before_joint.inflate(param_vec[0:6,:])
        self._after_joint.inflate(param_vec[6:12,:])
        self._gearing = param_vec[12,0]

    # Return column vector of config
    def deflate(self):
        param_vec = matrix(numpy.zeros((13,1), float))
        param_vec[0:6,0] = self._before_joint.deflate()
        param_vec[6:12,0] = self._after_joint.deflate()
        param_vec[12,0] = self._gearing
        return param_vec

    # Returns # of params needed for inflation & deflation
    def get_length(self):
        return 13

    def compute_pose(self, joint_pos):
        joint_T = SingleTransform([0, 0, 0, 0, joint_pos[0]*self._gearing, 0])
        return self._before_joint.transform * joint_T.transform * self._after_joint.transform

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
