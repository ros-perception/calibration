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

from calibration_estimation.urdf_params import UrdfParams
from calibration_estimation.single_transform import SingleTransform
import numpy
from numpy import array, matrix, zeros, cumsum, concatenate, reshape
import scipy.optimize
import sys
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg

class ErrorCalc:
    """
    Helpers for computing errors and jacobians
    """
    def __init__(self, robot_params, free_dict, multisensors, use_cov):
        self._robot_params = robot_params
        self._expanded_params = robot_params.deflate()
        self._free_list = robot_params.calc_free(free_dict)
        self._multisensors = multisensors
        self._use_cov = use_cov
        self._j_count = 0
        self.marker_pub = rospy.Publisher("pose_guesses", Marker)


    def calculate_full_param_vec(self, opt_param_vec):
        '''
        Take the set of optimization params, and expand it into the bigger param vec
        '''
        full_param_vec = self._expanded_params.copy()
        full_param_vec[numpy.where(self._free_list), 0] = opt_param_vec

        return full_param_vec

    def calculate_error(self, opt_all_vec):
        opt_param_vec, full_pose_arr = self.split_all(opt_all_vec)

        full_param_vec = self.calculate_full_param_vec(opt_param_vec)

        # Update the primitives with the new set of parameters
        self._robot_params.inflate(full_param_vec)

        # Update all the blocks' configs
        for multisensor in self._multisensors:
            multisensor.update_config(self._robot_params)

        r_list = []
        id = 0
        for multisensor, cb_pose_vec in zip(self._multisensors, list(full_pose_arr)):
            # Process cb pose
            cb_points = SingleTransform(cb_pose_vec).transform * self._robot_params.checkerboards[multisensor.checkerboard].generate_points()
            if (self._use_cov):
                r_list.append(multisensor.compute_residual_scaled(cb_points))
            else:
                r_list.append(multisensor.compute_residual(cb_points))

            cb_points_msgs = [ geometry_msgs.msg.Point(cur_pt[0,0], cur_pt[0,1], cur_pt[0,2]) for cur_pt in cb_points.T]   
            
            m = Marker()
            m.header.frame_id = self._robot_params.base_link
            m.ns = "points_3d"
            m.id = id
            m.type = Marker.SPHERE_LIST
            m.action = Marker.MODIFY
            m.points = cb_points_msgs
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.scale.x = 0.01
            m.scale.y = 0.01
            m.scale.z = 0.01
            self.marker_pub.publish(m)
            id += 1
                
        r_vec = concatenate(r_list)

        rms_error = numpy.sqrt( numpy.mean(r_vec**2) )
        
        print "\t\t\t\t\tRMS error: %.3f    \r" % rms_error,
        sys.stdout.flush()

        return array(r_vec)

    def calculate_jacobian(self, opt_all_vec):
        """
        Full Jacobian:
          The resulting returned jacobian can be thought of as a set of several concatenated jacobians as follows:

                [ J_multisensor_0 ]
            J = [ J_multisensor_1 ]
                [        :        ]
                [ J_multisensor_M ]

          where M is the number of multisensors, which is generally the number of calibration samples collected.

        Multisensor Jacobian:
          The Jacobian for a given multisensor is created by concatenating jacobians for each sensor

                            [ J_sensor_m_0 ]
          J_multisensor_m = [ J_sensor_m_1 ]
                            [      :       ]
                            [ J_sensor_m_S ]

          where S is the number of sensors in this multisensor.

        Sensor Jacobian:
          A single sensor jacobian is created by concatenating jacobians for the system parameters and checkerboards

          J_sensor_m_s = [ J_params_m_s | J_m_s_pose0 | J_m_s_pose1 | --- | J_m_s_CB_poseC ]

          The number of rows in J_sensor_m_s is equal to the number of rows in this sensor's residual.
          J_params_m_s shows how modifying the free system parameters affects the residual.
          Each J_m_s_pose_c is 6 columns wide and shows how moving a certain target affects the residual. All
            of the J_m_s_pose blocks are zero, except J_sensor_pose_m, since target m is the only target that
            was viewed by the sensors in this multisensor.
        """
        self._j_count += 1;
        sys.stdout.write("                     \r")
        sys.stdout.write("Computing Jacobian.. (cycle #%d)\r" % self._j_count)
        sys.stdout.flush()
        #import scipy.optimize.slsqp.approx_jacobian as approx_jacobian
        #J = approx_jacobian(opt_param_vec, self.calculate_error, 1e-6)

        opt_param_vec, full_pose_arr = self.split_all(opt_all_vec)

        # Allocate the full jacobian matrix
        ms_r_len = [ms.get_residual_length() for ms in self._multisensors]
        J = zeros([sum(ms_r_len), len(opt_all_vec)])

        # Calculate at which row each multisensor starts and ends
        ms_end_row = list(cumsum(ms_r_len))
        ms_start_row = [0] + ms_end_row[:-1]

        # Calculate at which column each multisensor starts and ends
        ms_end_col = list(cumsum([6]*len(self._multisensors)) + len(opt_param_vec))
        ms_start_col = [x-6 for x in ms_end_col]


        for i,ms in enumerate(self._multisensors):
            # Populate the parameter section for this multisensor
            J_ms_params = J[ ms_start_row[i]:ms_end_row[i],
                             0:len(opt_param_vec) ]
            s_r_len = [s.get_residual_length() for s in ms.sensors]
            s_end_row = list(cumsum(s_r_len))
            s_start_row = [0] + s_end_row[:-1]
            target_pose_T = SingleTransform(full_pose_arr[i,:]).transform
            # Fill in parameter section one sensor at a time
            for k,s in enumerate(ms.sensors):
                J_s_params = J_ms_params[ s_start_row[k]:s_end_row[k], :]
                J_s_params[:,:] = self.single_sensor_params_jacobian(opt_param_vec, target_pose_T, ms.checkerboard, s)

            # Populate the pose section for this multisensor
            J_ms_pose = J[ ms_start_row[i]:ms_end_row[i],
                           ms_start_col[i]:ms_end_col[i]]
            assert(J_ms_pose.shape[1] == 6)
            J_ms_pose[:,:] = self.multisensor_pose_jacobian(opt_param_vec, full_pose_arr[i,:], ms)

        sys.stdout.flush()

        return J

    def split_all(self, opt_all_vec):
        """
        Splits the input vector into two parts:
          1) opt_param_vec: The system parameters that we're optimizing,
          2) full_pose_arr: A Nx6 array where each row is a checkerboard pose
        """
        opt_param_len = sum(self._free_list)
        opt_param_vec = opt_all_vec[0:opt_param_len]

        full_pose_vec  = opt_all_vec[opt_param_len:]
        full_pose_arr = numpy.reshape(full_pose_vec, [-1,6])
        return opt_param_vec, full_pose_arr

    def single_sensor_params_jacobian(self, opt_param_vec, target_pose_T, target_id, sensor):
        """
        Computes the jacobian from the free system parameters to a single sensor
        Inputs:
        - opt_param_vec: Vector of the free system parameters (the parameters we're actuall optimizing)
        - target_pose_T: The pose of the target that this sensor is viewing
        - target_id: String name of the target that this sensor is viewing. This is necessary to generate the
                     set of feature points on the target
        - sensor: The actual sensor definition
        Output:
        - J_scaled: An mxn jacobian, where m is the length of sensor's residual and n is the length of opt_param_vec.
                    If covariance calculations are enabled, then the Jacobian is scaled by sqrt(Gamma), where Gamma
                    is the information matrix for this measurement.
        """
        sparsity_dict = sensor.build_sparsity_dict()
        required_keys = ['chains', 'tilting_lasers', 'transforms', 'rectified_cams', 'checkerboards']
        for cur_key in required_keys:
            if cur_key not in sparsity_dict.keys():
                sparsity_dict[cur_key] = {}
        # Generate the full sparsity vector
        full_sparsity_list = self._robot_params.calc_free(sparsity_dict)
        full_sparsity_vec = numpy.array(full_sparsity_list)

        # Extract the sparsity for only the parameters we are optimizing over

        opt_sparsity_vec = full_sparsity_vec[numpy.where(self._free_list)].copy()

        # Update the primitives with the new set of parameters
        full_param_vec = self.calculate_full_param_vec(opt_param_vec)
        self._robot_params.inflate(full_param_vec)
        sensor.update_config(self._robot_params)

        # based on code from scipy.slsqp
        x0 = opt_param_vec
        epsilon = 1e-6
        target_points = target_pose_T * self._robot_params.checkerboards[target_id].generate_points()
        f0 = sensor.compute_residual(target_points)
        if (self._use_cov):
            gamma_sqrt = sensor.compute_marginal_gamma_sqrt(target_points)
        Jt = numpy.zeros([len(x0),len(f0)])
        dx = numpy.zeros(len(x0))
        for i in numpy.where(opt_sparsity_vec)[0]:
            dx[i] = epsilon
            opt_test_param_vec = x0 + dx
            full_test_param_vec = self.calculate_full_param_vec(opt_test_param_vec)
            self._robot_params.inflate(full_test_param_vec)
            sensor.update_config(self._robot_params)
            target_points = target_pose_T * self._robot_params.checkerboards[target_id].generate_points()
            Jt[i] = (sensor.compute_residual(target_points) - f0)/epsilon
            dx[i] = 0.0
        J = Jt.transpose()
        if (self._use_cov):
            J_scaled = gamma_sqrt * J
        else:
            J_scaled = J
        return J_scaled

    def multisensor_pose_jacobian(self, opt_param_vec, pose_param_vec, multisensor):
        """
        Generates the jacobian from a target pose to the multisensor's residual.

        Input:
        - opt_param_vec: Vector of the free system parameters (the parameters we're actuall optimizing)
        - pose_param_vec: Vector of length 6 encoding the target's pose 0:3=translation 3:6=rotation_axis
        - multisensor: The actual multisensor definition.
        Output:
        - J_scaled: An mx6 jacobian, where m is the length of multisensor's residual. There are 6 columns since the
                    target's pose is encoded using 6 parameters.
                    If covariance calculations are enabled, then the Jacobian is scaled by sqrt(Gamma), where Gamma
                    is the information matrix for this measurement.
        """
        # Update the primitives with the new set of parameters
        full_param_vec = self.calculate_full_param_vec(opt_param_vec)
        self._robot_params.inflate(full_param_vec)
        multisensor.update_config(self._robot_params)
        cb_model = self._robot_params.checkerboards[multisensor.checkerboard]
        local_cb_points = cb_model.generate_points()

        # based on code from scipy.slsqp
        x0 = pose_param_vec
        epsilon = 1e-6
        world_pts = SingleTransform(x0).transform * local_cb_points
        f0 = multisensor.compute_residual(world_pts)
        if (self._use_cov):
            gamma_sqrt = multisensor.compute_marginal_gamma_sqrt(world_pts)

        Jt = numpy.zeros([len(x0),len(f0)])
        dx = numpy.zeros(len(x0))
        for i in range(len(x0)):
            dx[i] = epsilon
            test_vec = x0 + dx
            fTest = multisensor.compute_residual(SingleTransform(test_vec).transform * local_cb_points)
            Jt[i] = (fTest - f0)/epsilon
            #import code; code.interact(local=locals())
            dx[i] = 0.0
        J = Jt.transpose()
        if (self._use_cov):
            J_scaled = gamma_sqrt * J
        else:
            J_scaled = J
        return J_scaled

def build_opt_vector(robot_params, free_dict, pose_guess_arr):
    """
    Construct vector of all the parameters that we're optimizing over. This includes
    both the free system parameters and the checkerboard poses
    Inputs:
    - robot_params: Dictionary with all of the system parameters
    - free_dict: Dictionary that specifies which system parameters are free
    - pose_guess_arr: Mx6 array storing the current checkerboard poses, where M is
                      the number of calibration samples
    Returns: Vector of length (F + Mx6), where F is the number of 1's in the free_dict
    """

    # Convert the robot parameter dictionary into a vector
    expanded_param_vec = robot_params.deflate()
    free_list = robot_params.calc_free(free_dict)

    # Pull out only the free system parameters from the system parameter vector
    opt_param_vec = array(expanded_param_vec[numpy.where(free_list)].T)[0]

    assert(pose_guess_arr.shape[1] == 6)
    opt_pose_vec = reshape(pose_guess_arr, [-1])

    return numpy.concatenate([opt_param_vec, opt_pose_vec])

def compute_errors_breakdown(error_calc, multisensors, opt_pose_arr):
    errors_dict = {}
    # Compute the error for each sensor type
    for ms, k in zip(multisensors, range(len(multisensors))):
        cb = error_calc._robot_params.checkerboards[ms.checkerboard]
        cb_pose_vec = opt_pose_arr[k]
        target_pts = SingleTransform(cb_pose_vec).transform * cb.generate_points()
        for sensor in ms.sensors:
            r_sensor = sensor.compute_residual(target_pts) * numpy.sqrt(sensor.terms_per_sample)    # terms per sample is a hack to find rms distance, instead of a pure rms, based on each term
            if sensor.sensor_id not in errors_dict.keys():
                errors_dict[sensor.sensor_id] = []
            errors_dict[sensor.sensor_id].append(r_sensor)
    return errors_dict

def opt_runner(robot_params, pose_guess_arr, free_dict, multisensors, use_cov):
    """
    Runs a single optimization step for the calibration optimization.
      robot_params - Instance of UrdfParams
      free_dict - Dictionary storing which parameters are free
      multisensor - list of list of measurements. Each multisensor corresponds to a single checkerboard pose
      pose_guesses - List of guesses as to where all the checkerboard are. This is used to initialze the optimization
    """
    error_calc = ErrorCalc(robot_params, free_dict, multisensors, use_cov)

    # Construct the initial guess
    opt_all = build_opt_vector(robot_params, free_dict, pose_guess_arr)

    x, cov_x, infodict, mesg, iter = scipy.optimize.leastsq(error_calc.calculate_error, opt_all, Dfun=error_calc.calculate_jacobian, full_output=1)

    J = error_calc.calculate_jacobian(x)

    # A hacky way to inflate x back into robot params
    opt_param_vec, pose_vec = error_calc.split_all(x)
    expanded_param_vec = error_calc.calculate_full_param_vec(opt_param_vec)
    opt_pose_arr = reshape(pose_vec, [-1, 6])

    output_dict = error_calc._robot_params.params_to_config(expanded_param_vec)

    errors_dict = compute_errors_breakdown(error_calc, multisensors, opt_pose_arr)

    print ""
    print "Errors Breakdown:"
    for sensor_id, error_list in errors_dict.items():
        error_cat = numpy.concatenate(error_list)
        rms_error = numpy.sqrt( numpy.mean(error_cat**2) )
        print "  %s: %.6f" % (sensor_id, rms_error)

    return output_dict, opt_pose_arr, J

