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

import sys
import xml.dom

def update_joint(str_in, name, xyz=None, rpy=None, ref_shift=None):
    '''
    Updates the joint's reference position and rpy terms for the input xml string
    '''

    changelist = []

    joint_span = find_split_elem_span(str_in, 'joint', name)

    if joint_span is None:
        return None

    if rpy is not None:
        origin_span = find_atomic_elem_span(str_in, 'origin', *joint_span)
        rpy_span = find_attr_span(str_in, 'rpy', *origin_span)
        if rpy_span is not None:
            changelist.append((rpy_span, "%.10f %.10f %.10f" % tuple(rpy)))
        xyz_span = find_attr_span(str_in, 'xyz', *origin_span)
        if xyz_span is not None:
            changelist.append((xyz_span, "%.10f %.10f %.10f" % tuple(xyz)))

# TODO: bring this back?
#    if ref_shift is not None:
#        calibration_span = find_atomic_elem_span(str_in, 'calibration', *joint_span)
#        rising_span = find_attr_span(str_in, 'rising', *calibration_span)
#        falling_span = find_attr_span(str_in, 'falling', *calibration_span)
#        reference_position_span = find_attr_span(str_in, 'reference_position', *calibration_span)
#        for cur_cal_span in [rising_span, falling_span, reference_position_span]:
#            if cur_cal_span is not None:
#                orig_val = float(str_in[cur_cal_span[0]:cur_cal_span[1]])
#                changelist.append((cur_cal_span, "%.10f" % (orig_val + ref_shift)))

    return changelist

def update_transmission(str_in, name, reduction_scale):
    '''
    Updates a transmission
    new_reduction = original_recuction * reduction_scale
    '''
    changelist = []
    trans_span = find_split_elem_span(str_in, 'transmission', name)
    if trans_span is None:
        return None

    mech_red_span = find_split_elem_span(str_in, 'mechanicalReduction', None, *trans_span)
    if mech_red_span is None:
        return None

    internal_span = find_split_internals(str_in, *mech_red_span)
    if internal_span is None:
        return None

    reduction_orig = float(str_in[internal_span[0]:internal_span[1]])
    reduction_new  = reduction_orig * reduction_scale
    changelist.append( (internal_span, "%.12f" % reduction_new) )

    return changelist

def find_split_internals(str_in, start=0, end=None):
    '''
    Find everything enclosed by a split element tag
    '''
    if end is None:
        end = len(str_in)

    # Find the first '>'
    internals_start = str_in.find(">", start, end)+1

    # Keep walking along text until we get to the last '<'
    left_arrow = str_in.find('<', start, end)
    if left_arrow < 0:
        print "Error: Couldn't any '<' in text"
        print str_in[start:end]
        sys.exit(-1)

    while left_arrow >= 0:
        internals_end = left_arrow
        left_arrow = str_in.find('<', left_arrow+1, end)

    return (internals_start, internals_end)

def find_attr_span(str_in, name, start=0, end=None):
    '''
    Find the span of an attribute in an element.  The resulting start to (end+1) will
    display everything within the quotations.
    '''
    # Define end to be end of string
    if end is None:
        end = len(str_in)

    attr_types = ["%s=\"" % name,
                  "%s= \"" % name,
                  "%s =\"" % name,
                  "%s = \"" % name]

    for cur_type in attr_types:
        attr_name_start = str_in.find(cur_type, start, end)
        # See if we found the attribute
        if attr_name_start >= 0:
            attr_start = attr_name_start + len(cur_type)
            attr_end = str_in.find('\"', attr_start, end)
            if attr_end < 0:
                print "Error: Unmatch quotations:"
                print str_in[start:end]
                return None
            return (attr_start, attr_end)

    return None

def find_atomic_elem_span(str_in, node_name, start=0, end=None):
    '''
    Find the span of an element that is not 'split', like <elem atttr1="" attr2="" />, and notr <elem></elem>
    '''

    # Define end to be end of string
    if end is None:
        end = len(str_in)

    elem_start = str_in.find("<%s"%node_name, start, end)
    if elem_start < 0:
        print "Could not find [%s]" % node_name
        print str_in[elem_start:end]
        return None

    elem_end = str_in.find("/>", elem_start, end)
    if elem_end < 0:
        print "Error: Could not find end of [%s] elem" % node_name
        print str_in[elem_start:end]
        return None
    return (elem_start, elem_end + len("/>"))


def find_split_elem_span(str_in, elem_name, attr_name=None, start=0, end=None):
    '''
    Find the span of a xml node. This function assumes that it doesn't use an '/>'. Thus it should look like <elem> </elem>, and not <elem/>.
    '''
    if end is None:
        end = len(str_in)

    elem_start = str_in.find("<%s" % elem_name, start, end)
    while elem_start >= 0:
        elem_slash_end = str_in.find("/>", elem_start, end)
        elem_end = str_in.find(">", elem_start, end)
        if elem_end == -1:
            print "ERROR: Found [<%s] without matching close [>]" % elem_name
            print str_in[elem_start:]
            sys.exit(-1)

        # Continue, so long as we got to a '>' before getting to a '/>'
        if elem_slash_end == -1 or elem_end < elem_slash_end:
            #print "Found: %s" % str_in[elem_start:elem_end+1]
            block_delim = str_in.find("</%s>" % elem_name, elem_start, end)
            if block_delim < 0:
                print "ERROR: Didn't find </%s>" % elem_name
                print str_in[elem_start]
            block_delim += len("</%s>"%elem_name)

            # If we don't care about the name attribute, then we're already done
            if attr_name is None:
                break

            # Make sure it's not a gazebo elem
            if "%s:"%elem_name not in str_in[elem_start:block_delim]:
                #print "**** About to parse *****"
                #print str_in[elem_start:block_delim]
                joint_dom = xml.dom.minidom.parseString(str_in[elem_start:block_delim])
                #See if we're in the correct block
                if joint_dom.documentElement.getAttribute('name') == attr_name:
                    print "Found [%s]" % attr_name
                    break
        else:
            #print "Skipping: %s" % str_in[elem_start:elem_slash_end+2]
            pass

        elem_start = str_in.find("<%s"%elem_name, elem_start+1, end)

    if elem_start < 0:
        return None

    return elem_start, block_delim

