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

# A convenience object that wraps a joint_chain, along with transforms
# before and after the chain.
#
# The transforms can be thought of as follows:
#
#       before_chain_T[0] ... before_chain_T[N] -- chain -- after_chain_T[0] ... after_chain_T[N]
#      /
#   root

from sensor_msgs.msg import JointState
from numpy import matrix
import numpy

class FullChainRobotParams:

    # Initialize with dictionary of the current configuration.
    # Dictionary should have the following:
    #   - before_chain: List of transform ids to apply before the chain
    #   - chain_id: Chain ID for the joint_chain
    #   - link_num: Specifies which joint elem should be the last one to apply.
    #   - after_chain: List of transform ids to apply after the chain
    def __init__(self, config_dict):
        self._config_dict = config_dict
        self.calc_block = FullChainCalcBlock()

    def update_config(self, robot_params):
        before_chain_Ts = [robot_params.transforms[transform_name] for transform_name in self._config_dict["before_chain"]]
        if self._config_dict["chain_id"] == None:
            chain = None
            link_num = None
        else:
            chain       = robot_params.chains[ self._config_dict["chain_id"] ]
            link_num    = self._config_dict["link_num"]
        after_chain_Ts  = [robot_params.transforms[transform_name] for transform_name in self._config_dict["after_chain"]]
        self.calc_block.update_config(before_chain_Ts, chain, link_num, after_chain_Ts)

class FullChainCalcBlock:
    def update_config(self, before_chain_Ts, chain, link_num, after_chain_Ts):
        self._before_chain_Ts = before_chain_Ts
        self._chain = chain
        self._link_num = link_num
        self._after_chain_Ts = after_chain_Ts

    def fk(self, chain_state):
        pose = matrix(numpy.eye(4))

        # Apply the 'before chain' transforms
        for before_chain_T in self._before_chain_Ts:
            pose = pose * before_chain_T.transform

        # Apply the DH Chain
        if self._chain is not None:
            dh_T = self._chain.fk(chain_state, self._link_num)
            pose = pose * dh_T

        # Apply the 'after chain' transforms
        for after_chain_T in self._after_chain_Ts:
            pose = pose * after_chain_T.transform

        return pose

