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
# 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

param_names = ['spacing_x', 'spacing_y']

class Checkerboard:
    '''
    Primitive for generating 3D points of a checkerboard.
    '''

    # Dictionary with four elems
    #  num_x:   number of internal corners along x axis (Not configurable)
    #  num_y:   number of internal corners along y axis (Not configurable)
    #  width_x: spacing between corners along x direction
    #  width_y: spacing between corners along y direction
    def __init__(self, config = {'corners_x': 2,
                                 'corners_y': 2,
                                 'spacing_x': .10,
                                 'spacing_y': .10} ):
        rospy.logdebug('Initializing Checkerboard')
        self._corners_x = config['corners_x']
        self._corners_y = config['corners_y']

        param_vec = reshape( matrix([ config['spacing_x'], config['spacing_y'] ], float), (-1,1))
        self.inflate(param_vec)

    def calc_free(self, free_config):
        return [free_config[x] == 1 for x in param_names]

    def params_to_config(self, param_vec):
        return { 'corners_x' : self._corners_x,
                 'corners_y' : self._corners_y,
                 'spacing_x' : float(param_vec[0,0]),
                 'spacing_y' : float(param_vec[1,0]) }

    # Convert column vector of params into config
    def inflate(self, param_vec):
        self._spacing_x = param_vec[0,0]
        self._spacing_y = param_vec[1,0]

    # Return column vector of config
    def deflate(self):
        param_vec = reshape(matrix( [self._spacing_x, self._spacing_y], float ), (-1,1))
        return param_vec

    # Returns # of params needed for inflation & deflation
    def get_length(self):
        return len(param_names)

    # Generate the 3D points associated with all the corners of the checkerboard
    # returns - 4xN numpy matrix with all the points (in homogenous coords)
    def generate_points(self):
        N = self._corners_x * self._corners_y
        pts = matrix(numpy.zeros((4,N), float))
        for y in range(0,self._corners_y):
            for x in range(0,self._corners_x):
                n = y*self._corners_x + x
                pts[:,n] = numpy.matrix([ [(x-(self._corners_x-1)/2) * self._spacing_x],
                                          [-1*(y-(self._corners_y-1)/2) * self._spacing_y],
                                          [0],
                                          [1] ])
        return pts

