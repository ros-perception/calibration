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

# This was formerly a DhChain

class JointChain:
    def __init__(self, config):
        # Determine number of links
        self._M = len(config['transforms'])
        rospy.logdebug("Initializing joint chain with [%u] links", self._M)

        param_mat = numpy.matrix([ [eval(str(x)) for x in y] for y in config['transforms']], float)
        self._axis = numpy.matrix([ eval(str(x)) for x in config['axis']], float)
        assert(self._M*6 == param_mat.size)
        self._length = param_mat.size       # The number of params needed to configure this chain
        self._config = param_mat            # Mx6 matrix. Each row is a link, containing [x,y,z,r,p,y]

        self._cov_dict = config['cov']
        self._gearing = config['gearing']
        assert( len(self._cov_dict['joint_angles']) == self._M)

    def calc_free(self, free_config):
        assert( len(free_config['transforms']) == self._M)
        assert( len(free_config['axis']) == self._M)
        assert( len(free_config['gearing']) == self._M )

        # Flatten the config
        flat_config = sum(free_config['transforms'], [])

        #print len(flat_config), self._M*6
        assert( len(flat_config) == self._M * 6)

        flat_config = flat_config + free_config['gearing']

        # Convert int list into bool list
        flat_free = [x == 1 for x in flat_config]

        return flat_free

    def params_to_config(self, param_vec):
        assert(param_vec.shape == (self._M * 7, 1)) # 6 offset params + 1 gearing per joint
        dh_param_vec = param_vec[0:(self._M * 6),0]
        gearing_param_vec = param_vec[(self._M * 6):,0]

        param_mat = reshape( dh_param_vec.T, (-1,6))
        config_dict = {}
        config_dict['transforms'] = param_mat.tolist()
        config_dict['axis'] = self._axis.tolist()
        config_dict['gearing'] = (array(gearing_param_vec)[:,0]).tolist()
        config_dict['cov'] = self._cov_dict
        return config_dict

    # Convert column vector of params into config
    def inflate(self, param_vec):
        #print param_vec.size
        param_mat = param_vec[:self._M*6,:]
        self._config = reshape(param_mat, (-1,6))
        self._gearing = array(param_vec[self._M*6:,:])[:,0].tolist()

    # Return column vector of config
    def deflate(self):
        #print self._config.size
        param_vec = reshape(self._config, (-1,1))
        param_vec = numpy.concatenate([param_vec, matrix(self._gearing).T])
        #print param_vec.size
        return param_vec

    # Returns # of params needed for inflation & deflation
    def get_length(self):
        return self._M*7

    # Returns 4x4 numpy matrix of the pose of the tip of
    # the specified link num. Assumes the last link's tip
    # when link_num < 0
    def fk(self, chain_state, link_num=-1):
        if link_num < 0:
            link_num = self._M

        dh_trimmed  = self._config[0:(link_num+1), :]
        pos_trimmed = chain_state.position[0:(link_num+1)]
        gearing_trimmed = self._gearing[0:(link_num+1)]

        pos_scaled = [cur_pos * cur_gearing for cur_pos, cur_gearing in zip(pos_trimmed, gearing_trimmed)]

        eef_pose = chain_T(dh_trimmed, pos_scaled, self._axis)
        return eef_pose

# Computes the transform for a chain
# params: Mx4 matrix, where M is the # of links in the model
#         Each row represents a link transform [x,y,z,r,p,y]
# joint_pos:
# returns: 4x4 numpy matrix
def chain_T(params, joint_pos, axis):
    if numpy.mod(params.size,6) != 0:
        raise Exception("input size must be of length multiple of 6. Current is %s = %u" % (params.shape, params.size))
    M = params.size/6

    if len(joint_pos) != M:
        raise Exception("joint_positions must length must be [%u]. Currently is [%u]" % (M, len(joint_pos)))

    # Reshape input to be Nx4
    chain_final = numpy.array(params.copy())
    chain_final.shape = (M, 6)

    for m in range(0,M):
        k = int(axis[0,m])-1
        chain_final[m,k] = chain_final[m,k]+joint_pos[m]

    # Mutliply all the dh links together (T0 * T1 * T2 * ... * TN)
    return reduce( lambda x,y: x*y, [ link_T(r[0]) for r in vsplit(chain_final, M)] )

# Computes the 4x4 transformation matrix for a given set of link parameters.
# link_params - params of the form (x,y,z, r,p,y)
# Output, a 4x4 transform from link (i-1) to (i)
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

    return T_roll * T_pitch * T_yaw * T_trans

