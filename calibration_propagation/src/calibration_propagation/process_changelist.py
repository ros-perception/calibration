#! /usr/bin/env python

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

import copy

def process_changelist(changelist, str_in):
    '''
    Given a changelist, updates the specified parts of str_in.
    changelist - List of change requests, each of the form ((start, end), new_text)
    '''

    cl_sorted = copy.copy(changelist)
    cl_sorted.sort(lambda x,y:cmp(x[0][0], y[0][0]))

    for k in range(len(cl_sorted)-1):
        if cl_sorted[k][0][1] > cl_sorted[k+1][0][0]:
            print "Ordering error between %s and %s" % (cl_sorted[k][0], cl_sorted[k+1][0])
            assert False

    str_out = ""
    cur_pos = 0
    for cur_span, new_text in cl_sorted:
        cur_start = cur_span[0]
        cur_end   = cur_span[1]
        str_out = str_out + str_in[cur_pos:cur_start] + new_text
        cur_pos = cur_end

    str_out = str_out + str_in[cur_pos:]

    return str_out
