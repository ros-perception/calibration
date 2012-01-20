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
from numpy import matrix, array, vsplit, sin, cos, reshape, ones, sqrt
import rospy

param_names = ['baseline_shift', 'f_shift', 'cx_shift', 'cy_shift']

class RectifiedCamera:
    '''
    Primitive for projecting 3D points into a monocular camera. The baseline
    term of the projection matrix can be changed.  The baseline shift is added
    to the P[0,3] term of the projection matrix found in CameraInfo
    Parameters are ordered as follows [baseline_shift, f_shift, cx_shift, cy_shift]
    '''
    def __init__(self, config):
        rospy.logdebug('Initializng rectified camera')
        self._config = config
        self._cov_dict = config['cov']
        try:
            self._rgbd = config["baseline_rgbd"] != 0.0
        except:
            self._rgbd = None

    def calc_free(self, free_config):
        return [free_config[x] == 1 for x in param_names]

    def params_to_config(self, param_vec):
        param_list = array(param_vec)[:,0].tolist()
        param_dict = dict(zip(param_names, param_list))
        param_dict['cov'] = self._cov_dict
        if self._rgbd:
            param_dict['baseline_rgbd'] = self._rgbd
        param_dict['frame_id'] = self._config['frame_id']
        param_dict['chain_id'] = self._config['chain_id']   # TODO: kill this
        return param_dict

    # Convert column vector of params into config, expects a 4x1 matrix
    def inflate(self, param_vec):
        param_list = array(param_vec)[:,0].tolist()
        for x,y in zip(param_names, param_list):
            self._config[x] = y

    # Return column vector of config. In this case, it's always a 4x1 matrix
    def deflate(self):
        return matrix([self._config[x] for x in param_names]).T

    # Returns # of params needed for inflation & deflation
    def get_length(self):
        return len(param_names)

    def get_param_names(self):
        return param_names

    # Project a set of 3D points points into pixel coordinates
    # P_list - Projection matrix. We expect this to be a 1x12 list. We then reshape
    #          it into a 3x4 matrix (by filling 1 row at a time)
    # pts - 4xN numpy matrix holding the points that we want to project (homogenous coords)
    def project(self, P_list, pts):
        N = pts.shape[1]

        # Reshape P_list into an actual matrix
        P = reshape( matrix(P_list, float), (3,4) )

        # Update the baseline by the "baseline_shift"
        P[0,3] = P[0,3] + self._config['baseline_shift']
        P[0,0] = P[0,0] + self._config['f_shift']
        P[1,1] = P[1,1] + self._config['f_shift']
        P[0,2] = P[0,2] + self._config['cx_shift']
        P[1,2] = P[1,2] + self._config['cy_shift']

        if (pts.shape[0] == 3):
            rospy.logfatal('Got vector of points with only 3 rows. Was expecting at 4 rows (homogenous coordinates)')

        # Apply projection matrix
        pixel_pts_h = P * pts

        # Strip out last row (3rd) and rescale
        if self._rgbd:
            pixel_pts = pixel_pts_h[0:3,:] / pixel_pts_h[2,:]
            for i in range(N):
                d = sqrt( (pts[0,i] * pts[0,i]) + (pts[1,i] * pts[1,i]) + (pts[2,i] * pts[2,i]) )
                pixel_pts[2,i] = self.get_disparity(P,d)
        else:
            pixel_pts = pixel_pts_h[0:2,:] / pixel_pts_h[2,:]

        return pixel_pts

    def get_disparity(self, P_list, distance_to_point):
        P = reshape( matrix(P_list, float), (3,4) )
        return (P[0,0] * self._config["baseline_rgbd"]) / distance_to_point
        

