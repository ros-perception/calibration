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
from numpy import matrix, vsplit, sin, cos, reshape, zeros, pi
import rospy

class SingleTransform:
    def __init__(self, config = [0, 0, 0, 0, 0, 0]):
        assert(len(config) == 6)

        eval_config = [eval(str(x)) for x in config]

        self._config = reshape(matrix(eval_config, float), (-1,1))

        rospy.logdebug("Initializing single transform with params [%s]", ", ".join(["% 2.4f" % x for x in eval_config]))
        self.inflate(self._config)

    def calc_free(self, free_config):
        assert( len(free_config) == 6 )
        return [x == 1 for x in free_config]


    def params_to_config(self, param_vec):
        assert(param_vec.shape == (6,1))
        return param_vec.T.tolist()[0]

    # Convert column vector of params into config
    def inflate(self, p):
        assert(p.size == 6)

        self._config = p.copy()  # Once we can back compute p from T, we don't need _config

        # Init output matrix
        T = matrix( zeros((4,4,), float))
        T[3,3] = 1.0
        
        # Copy position into matrix
        T[0:3,3] = p[0:3,0]
        
        # Renormalize the rotation axis to be unit length
        U,S,Vt = numpy.linalg.svd(p[3:6,0])
        a = U[:,0]
        rot_angle = S[0]*Vt[0,0]
        
        # Build rotation matrix
        c = cos(rot_angle)
        s = sin(rot_angle)
        R = matrix( [ [   a[0,0]**2+(1-a[0,0]**2)*c, a[0,0]*a[1,0]*(1-c)-a[2,0]*s, a[0,0]*a[2,0]*(1-c)+a[1,0]*s],
                      [a[0,0]*a[1,0]*(1-c)+a[2,0]*s,    a[1,0]**2+(1-a[1,0]**2)*c, a[1,0]*a[2,0]*(1-c)-a[0,0]*s],
                      [a[0,0]*a[2,0]*(1-c)-a[1,0]*s, a[1,0]*a[2,0]*(1-c)+a[0,0]*s,    a[2,0]**2+(1-a[2,0]**2)*c] ] )

        T[0:3,0:3] = R
        self.transform = T

    # Take transform, and convert into 6 param vector
    def deflate(self):
        # todo: This currently a hacky stub. To be correct, this should infer the parameter vector from the 4x4 transform
        return self._config

    # Returns # of params needed for inflation & deflation
    def get_length(self):
        return 6

