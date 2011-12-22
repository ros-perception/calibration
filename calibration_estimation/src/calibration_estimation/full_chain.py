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

from sensor_msgs.msg import JointState
from numpy import matrix
import numpy

class FullChainRobotParams:
    '''
    Wraps a full set of transforms, including a joint_chain

      root
          \    
           fixed links before -- chain -- fixed links after
    '''
    def __init__(self, chain_id, tip, root=None):
        self.chain_id = chain_id
        self.root = root
        self.tip = tip
        self.calc_block = FullChainCalcBlock()

    def update_config(self, robot_params):
        if self.root == None: 
            self.root = robot_params.base_link
        try:
            chain = robot_params.chains[self.chain_id]
            before_chain = robot_params.urdf.get_chain(self.root, chain.root, links=False)
            full_chain = robot_params.urdf.get_chain(chain.root, chain.tip)
            try:
                after_chain = robot_params.urdf.get_chain(chain.tip, self.tip, links=False)
                link_num = -1
            except:
                # using only part of the chain, have to calculate link_num
                tip_chain = robot_params.urdf.get_chain(chain.root,self.tip)
                new_root = full_chain[0]
                i = 1
                link_num = -1
                while i < len(full_chain):
                    if full_chain[i] in tip_chain:
                        if full_chain[i] in chain._active:
                            link_num += 1
                            new_root = full_chain[i+1]
                    i += 1
                after_chain = robot_params.urdf.get_chain(new_root, self.tip, links=False)
        except KeyError:
            chain = None
            before_chain = robot_params.urdf.get_chain(self.root, self.tip, links=False)
            after_chain = []
            link_num = None
        before_chain_Ts = [robot_params.transforms[transform_name] for transform_name in before_chain]
        after_chain_Ts  = [robot_params.transforms[transform_name] for transform_name in after_chain]
        self.calc_block.update_config(before_chain_Ts, chain, link_num, after_chain_Ts)

    def build_sparsity_dict(self):
        """
        Build a dictionary that defines which parameters will in fact affect a measurement for a sensor using this chain.
        """
        sparsity = dict()
        sparsity['transforms'] = {}
        sparsity['chains'] = {}
        if self.chain_id is not None:
            for cur_transform in ( self.calc_block._before_chain_Ts + \
                                   self.calc_block._chain._transforms.values() + \
                                   self.calc_block._after_chain_Ts ):
                sparsity['transforms'][cur_transform._name] = [1, 1, 1, 1, 1, 1]
            sparsity['chains'][self.chain_id] = {}
            link_num = self.calc_block._link_num
            if link_num < 0:
                link_num = self.calc_block._chain._M
            sparsity['chains'][self.chain_id]['gearing'] = [1 for i in range(link_num)]
        else:
            for cur_transform in ( self.calc_block._before_chain_Ts + \
                                   self.calc_block._after_chain_Ts ):
                sparsity['transforms'][cur_transform._name] = [1, 1, 1, 1, 1, 1]
        return sparsity

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

        # Apply the Chain
        if self._chain is not None:
            chain_T = self._chain.fk(chain_state, self._link_num)
            pose = pose * chain_T

        # Apply the 'after chain' transforms
        for after_chain_T in self._after_chain_Ts:
            pose = pose * after_chain_T.transform

        return pose

