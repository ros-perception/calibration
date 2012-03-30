#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008-2012, Willow Garage, Inc.
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

# author: Vijay Pradeep, Michael Ferguson

import roslib; roslib.load_manifest('calibration_estimation')

import sys
import rospy
import time
import numpy
import rosbag
import yaml
import os.path
import numpy
from numpy import kron, ones, eye, array, matrix, diag
from calibration_estimation.cal_bag_helpers import *
import calibration_estimation.multi_step_cov_estimator as est_helpers
import calibration_estimation.opt_runner as opt_runner
from calibration_estimation.sensors.multi_sensor import MultiSensor
from calibration_estimation.urdf_params import UrdfParams
from calibration_estimation.single_transform import SingleTransform
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg

def usage():
    rospy.logerr("Not enough arguments")
    print "Usage:"
    print " ./error_visualization.py [bagfile] [output_dir] [loop_list.yaml]"
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node("error_visualization")
    marker_pub = rospy.Publisher("calibration_error", Marker)

    print "Starting The Error Visualization App\n"

    if (len(rospy.myargv()) < 3):
        usage()
    else:
        bag_filename = rospy.myargv()[1]
        output_dir = rospy.myargv()[2]
        loop_list_filename = rospy.myargv()[3]

    print "Using Bagfile: %s\n" % bag_filename
    if not os.path.isfile(bag_filename):
        rospy.logerr("Bagfile does not exist. Exiting...")
        sys.exit(1)

    loop_list = yaml.load(open(loop_list_filename))
    robot_description = get_robot_description(bag_filename)

    config_param_name = "calibration_config"
    if not rospy.has_param(config_param_name):
        rospy.logerr("Could not find parameter [%s]. Please populate this namespace with the estimation configuration.", config_param_name)
        sys.exit(1)
    config = rospy.get_param(config_param_name)

    # Process all the sensor definitions that are on the parameter server
    sensors_name = "sensors"
    if sensors_name not in config.keys():
        rospy.logerr("Could not find namespace [%s/%s]. Please populate this namespace with sensors.", (config_param_name, sensors_name))
        sys.exit(1)
    all_sensors_dict = est_helpers.build_sensor_defs(config[sensors_name])
    all_sensor_types = list(set([x['sensor_type'] for x in all_sensors_dict.values()]))

    # Load all the calibration steps.
    step_list = est_helpers.load_calibration_steps(config["cal_steps"])
    # Only process the last step
    cur_step = step_list[-1]

    # Load the resulting system definition
    system_def_dict = yaml.load(open(output_dir + "/" + cur_step["output_filename"] + ".yaml"))
    system_def = UrdfParams(robot_description, system_def_dict)
    cb_poses = yaml.load(open(output_dir + "/" + cur_step["output_filename"] + "_poses.yaml"))
    free_dict = yaml.load(cur_step["free_params"])

    sample_skip_list = rospy.get_param('calibration_skip_list', [])
    scatter_list = []
    marker_count = 0
    label_list = list()
    for cur_loop in loop_list:
        marker_count += 1

        sensor_defs = est_helpers.load_requested_sensors(all_sensors_dict, [cur_loop['cam'], cur_loop['3d']])
        multisensors = get_multisensors(bag_filename, sensor_defs, sample_skip_list)

        if (len([ms for ms in multisensors if len(ms.sensors) == 2]) == 0):
            print "********** No Data for [%s] + [%s] pair **********" % (cur_loop['cam'], cur_loop['3d'])
            continue
        label_list.append(cur_loop)

        # Only grab the samples that have both the requested cam and requested 3D sensor
        multisensors_pruned, cb_poses_pruned = zip(*[(ms,cb) for ms,cb in zip(multisensors, cb_poses) if len(ms.sensors) == 2])
        sample_ind = [k for k,ms in zip(range(len(multisensors)), multisensors) if len(ms.sensors) == 2]
        sample_ind = [i for i in sample_ind if i not in sample_skip_list]

        print "Sample Indices:"
        print ", ".join(["%u" % i for i in sample_ind])

        for ms in multisensors:
            ms.update_config(system_def)

        error_calc = opt_runner.ErrorCalc(system_def, free_dict, multisensors_pruned, False)
        opt_all_vec = opt_runner.build_opt_vector(system_def, free_dict, numpy.array(cb_poses_pruned))
        e = error_calc.calculate_error(opt_all_vec)

        # Display error breakdown
        errors_dict = opt_runner.compute_errors_breakdown(error_calc, multisensors_pruned, numpy.array(cb_poses_pruned))

        print ""
        print "Errors Breakdown:"
        for sensor_id, error_list in errors_dict.items():
            print "  %s:" % sensor_id
            i = 0
            for error in error_list:
                if i in sample_ind:        
                    rms_error = numpy.sqrt( numpy.mean(error**2) )
                    print "    Sample %d: %.6f" % (i, rms_error)
                i += 1
            error_cat = numpy.concatenate(error_list)
            rms_error = numpy.sqrt( numpy.mean(error_cat**2) )
            print "    Total: %.6f" % rms_error

        # Calculate loop errors
        chain_sensors = [[s for s in ms.sensors if s.sensor_id == cur_loop['3d']][0]  for ms in multisensors_pruned]
        cam_sensors   = [[s for s in ms.sensors if s.sensor_id == cur_loop['cam']][0] for ms in multisensors_pruned]
        fk_points = [s.get_measurement() for s in chain_sensors]
        cb_points = [SingleTransform(pose).transform * system_def.checkerboards[ms.checkerboard].generate_points() for pose, ms in zip(cb_poses_pruned,multisensors_pruned)]

        points_list_fk    = [ geometry_msgs.msg.Point(cur_pt[0, 0], cur_pt[0, 1], cur_pt[0, 2]) for cur_pt in list(numpy.concatenate(fk_points,1).T)]
        points_list_guess = [ geometry_msgs.msg.Point(cur_pt[0, 0], cur_pt[0, 1], cur_pt[0, 2]) for cur_pt in list(numpy.concatenate(cb_points,1).T)]

        m = Marker()
        m.header.frame_id = system_def.base_link
        m.ns = "points_3d"
        m.id = marker_count
        m.type = Marker.SPHERE_LIST
        m.action = Marker.MODIFY
        m.points = points_list_fk
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.scale.x = 0.01
        m.scale.y = 0.01
        m.scale.z = 0.01
        marker_pub.publish(m)

        m.points = points_list_guess
        m.ns = "points_cam"
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        marker_pub.publish(m)

        cam_Js   = [s.compute_expected_J(fk) for s,fk in zip(cam_sensors, fk_points)]
        cam_covs = [matrix(array(s.compute_cov(fk)) * kron(eye(s.get_residual_length()/2),ones([2,2]))) for s,fk in zip(cam_sensors, fk_points)]
        fk_covs  = [matrix(array(s.compute_cov(None)) * kron(eye(s.get_residual_length()/3),ones([3,3]))) for s in chain_sensors]

        full_covs = [matrix(cam_J)*fk_cov*matrix(cam_J).T + cam_cov for cam_J, cam_cov, fk_cov in zip(cam_Js, cam_covs, fk_covs)]

        proj_points = [s.compute_expected(pts) for (s,pts) in zip(cam_sensors,fk_points)]
        meas_points = [s.get_measurement() for s in cam_sensors]

        r = numpy.concatenate(proj_points) - numpy.concatenate(meas_points)

        import matplotlib.pyplot as plt
        cur_scatter = plt.scatter(array(r)[:,0], array(r)[:,1], **cur_loop['plot_ops'])
        scatter_list.append(cur_scatter)

    plt.axis('equal')
    plt.grid(True)
    plt.legend(scatter_list, [x['name'] for x in label_list], prop={'size':'x-small'})
    plt.show()

    rospy.spin()
    sys.exit(0)

