# Software License Agreement (BSD License)
#
# Copyright (c) 2008-2011, Willow Garage, Inc.
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
from numpy import matrix, vsplit, sin, cos, reshape, pi, array
import rospy

class JointChain:
    '''
    This represents a chain of actuated joints
    parameters = [gearing*num_of_joints]
    '''
    def __init__(self, config):
        self.root = config['root']
        self.tip = config['tip']
        self._joints = config['joints']
        self._active = config['active_joints']
        self._M = len(config['active_joints'])

        rospy.logdebug("Initializing joint chain with [%u] links", self._M)

        self._transforms = config['transforms']
        self._axis = numpy.matrix([ eval(str(x)) for x in config['axis']], float)
        self._cov_dict = config['cov']
        self._gearing = config['gearing']

    def calc_free(self, free_config):
        return [x == 1 for x in free_config['gearing']]

    def params_to_config(self, param_vec):
        config_dict = {}
        config_dict['axis'] = self._axis.tolist()
        config_dict['gearing'] = (array(param_vec)[:,0]).tolist()
        config_dict['cov'] = self._cov_dict
        config_dict['root'] = self.root
        config_dict['tip'] = self.tip
        return config_dict

    # Convert column vector of params into config
    def inflate(self, param_vec):
        self._gearing = array(param_vec)[:,0].tolist()

    # Return column vector of config
    def deflate(self):
        return matrix(self._gearing).T

    # Returns # of params needed for inflation & deflation
    def get_length(self):
        return self._M

    # Returns 4x4 numpy matrix of the pose of the tip of
    # the specified link num. Assumes the last link's tip
    # when link_num < 0
    def fk(self, chain_state, link_num=-1):
        if link_num < 0:
            link_num = self._M

        pos_trimmed = chain_state.position[0:(link_num+1)]
        gearing_trimmed = self._gearing[0:(link_num+1)]

        pos_scaled = [cur_pos * cur_gearing for cur_pos, cur_gearing in zip(pos_trimmed, gearing_trimmed)]
        
        T = matrix([ [ 1, 0, 0, 0 ],
                     [ 0, 1, 0, 0 ],
                     [ 0, 0, 1, 0 ],
                     [ 0, 0, 0, 1 ] ])

        m = 0
        for joint_name in self._joints:
            if m > link_num:
                break
            transform = self._transforms[joint_name]
            if joint_name in self._active:
                k = int(self._axis[0,m])
                p = transform.deflate()
                p = p.T.tolist()[0]
                if k > 0:
                    p[k-1] = p[k-1] + pos_scaled[m]
                else:
                    p[abs(k)-1] = p[abs(k)-1] - pos_scaled[m]
                T = T * link_T( p )
                m += 1
            else:
                T = T * transform.transform                

        return T

def link_T(link_params):
    r = link_params[3]
    p = link_params[4]
    y = link_params[5]

    T_roll =  matrix([ [ 1,   0,       0,    0 ],
                       [ 0, cos(r), -sin(r), 0 ],
                       [ 0, sin(r),  cos(r), 0 ],
                       [ 0,   0,       0,    1 ] ])

    T_pitch = matrix([ [  cos(p), 0, sin(p), 0 ],
                       [    0,    1,   0,    0 ],
                       [ -sin(p), 0, cos(p), 0 ],
                       [    0,    0,   0,    1 ] ])

    T_yaw =   matrix([ [ cos(y), -sin(y), 0, 0 ],
                       [ sin(y),  cos(y), 0, 0 ],
                       [   0,       0,    1, 0 ],
                       [   0,       0,    0, 1 ] ])


    T_trans = matrix([ [ 1, 0, 0, link_params[0] ],
                       [ 0, 1, 0, link_params[1] ],
                       [ 0, 0, 1, link_params[2] ],
                       [ 0, 0, 0, 1 ] ])

    return T_trans * T_roll * T_pitch * T_yaw

