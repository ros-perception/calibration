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

import roslib; roslib.load_manifest('calibration_estimation')

import sys
import rospy
import numpy

from calibration_estimation.joint_chain import JointChain
from calibration_estimation.tilting_laser import TiltingLaser
from calibration_estimation.single_transform import SingleTransform
from calibration_estimation.camera import RectifiedCamera
from calibration_estimation.checkerboard import Checkerboard

# Construct a dictionary of all the primitives of the specified type
def init_primitive_dict(start_index, config_dict, PrimitiveType):
    cur_index = start_index

    # Init the dictionary to store the constructed primitives
    primitive_dict = dict()

    # Process each primitive in the list of configs
    for key, elem in config_dict.items():
        #import code; code.interact(local=locals())
        primitive_dict[key] = PrimitiveType(elem)
        # Store lookup indices
        primitive_dict[key].start = cur_index
        primitive_dict[key].end = cur_index + primitive_dict[key].get_length()
        cur_index = primitive_dict[key].end

    # Return the primitives, as well as the end index
    return primitive_dict, cur_index

# Given a dictionary of initialized primitives, inflate
# their configuration, given a parameter vector
def inflate_primitive_dict(param_vec, primitive_dict):
    for key, elem in primitive_dict.items():
        elem.inflate(param_vec[elem.start:elem.end,0])

# Given a dictionary of initialized primitives, deflate
# their configuration, into the specified parameter vector
def deflate_primitive_dict(param_vec, primitive_dict):
    for key, elem in primitive_dict.items():
        #print elem, elem.start, elem.end
        param_vec[elem.start:elem.end,0] = elem.deflate()

# Iterate over config dictionary and determine which parameters should be free,
# based on the the flags in the free dictionary. Once computed, update the part
# of the target vector that corresponds to this primitive
def update_primitive_free(target_list, config_dict, free_dict):
    for key, elem in config_dict.items():
        if key in free_dict.keys():
            free = elem.calc_free(free_dict[key])
            target_list[elem.start:elem.end] = free

# Given a parameter vector, Compute the configuration of all the primitives of a given type
def primitive_params_to_config(param_vec, primitive_dict):
    config_dict = dict()
    for key, elem in primitive_dict.items():
        config_dict[key] = elem.params_to_config(param_vec[elem.start:elem.end,0])
    return config_dict

# Stores the current configuration of all the primitives in the system
class RobotParams:
    def __init__(self):
        pass

    def configure(self, config_dict):

        # Where the next primitive should stored
        cur_index = 0

        self.chains,         cur_index = init_primitive_dict(cur_index, config_dict["chains"],      JointChain)
        try: self.tilting_lasers, cur_index = init_primitive_dict(cur_index, config_dict["tilting_lasers"], TiltingLaser)
        except: self.tilting_lasers = dict()
        self.transforms,     cur_index = init_primitive_dict(cur_index, config_dict["transforms"],     SingleTransform)
        self.rectified_cams, cur_index = init_primitive_dict(cur_index, config_dict["rectified_cams"], RectifiedCamera)
        self.checkerboards,  cur_index = init_primitive_dict(cur_index, config_dict["checkerboards"],   Checkerboard)

        self.length = cur_index

    def calc_free(self, free_dict):
        # Initilize free list
        free_list = [0] * self.length
        #import code; code.interact(local=locals())
        update_primitive_free(free_list, self.chains,      free_dict["chains"])
        try: update_primitive_free(free_list, self.tilting_lasers, free_dict["tilting_lasers"])
        except: pass
        update_primitive_free(free_list, self.transforms,     free_dict["transforms"])
        update_primitive_free(free_list, self.rectified_cams, free_dict["rectified_cams"])
        update_primitive_free(free_list, self.checkerboards,  free_dict["checkerboards"])
        return free_list

    def params_to_config(self, param_vec):
        assert(self.length == param_vec.size)
        config_dict = dict()
        config_dict["chains"]         = primitive_params_to_config(param_vec, self.chains)
        config_dict["tilting_lasers"] = primitive_params_to_config(param_vec, self.tilting_lasers)
        config_dict["transforms"]     = primitive_params_to_config(param_vec, self.transforms)
        config_dict["rectified_cams"] = primitive_params_to_config(param_vec, self.rectified_cams)
        config_dict["checkerboards"]  = primitive_params_to_config(param_vec, self.checkerboards)
        return config_dict

    def inflate(self, param_vec):
        assert(self.length == param_vec.size)

        inflate_primitive_dict(param_vec, self.chains)
        inflate_primitive_dict(param_vec, self.tilting_lasers)
        inflate_primitive_dict(param_vec, self.transforms)
        inflate_primitive_dict(param_vec, self.rectified_cams)
        inflate_primitive_dict(param_vec, self.checkerboards)

    def deflate(self):
        param_vec = numpy.matrix( numpy.zeros((self.length,1), float))
        deflate_primitive_dict(param_vec, self.chains)
        deflate_primitive_dict(param_vec, self.tilting_lasers)
        deflate_primitive_dict(param_vec, self.transforms)
        deflate_primitive_dict(param_vec, self.rectified_cams)
        deflate_primitive_dict(param_vec, self.checkerboards)
        return param_vec

