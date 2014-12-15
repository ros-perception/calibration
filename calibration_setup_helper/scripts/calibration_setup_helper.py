#!/usr/bin/env python

# Copyright (C) 2014 Kei Okada
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import rospy
import argparse
import os,sys,itertools
from urdf_parser_py.urdf import URDF

class CalibrationSetupHelper:
    def __init__(self, robot_description, base_link, arm_root_link, arm_tip_link, head_root_link, head_tip_link, arm_controller, head_controller, head_camera_frame, head_camera_joint, camera_namespace):
        self.robot_description = robot_description
        self.base_link = base_link
        self.arm_root_link = arm_root_link
        self.arm_tip_link = arm_tip_link
        self.head_root_link = head_root_link
        self.head_tip_link = head_tip_link
        self.arm_controller = arm_controller
        self.head_controller = head_controller
        self.head_camera_frame = head_camera_frame
        self.head_camera_joint = head_camera_joint
        self.camera_namespace = camera_namespace

        self.robot = URDF().parse(self.robot_description)
        self.robot_name = self.robot.name.lower()

        if not self.base_link:
            self.base_link = self.robot.get_root()
        if self.robot.link_map.has_key(self.base_link) == False:
            self.error_tip_link(self.base_link, '--base-link')
        if self.robot.link_map.has_key(self.arm_root_link) == False:
            self.error_tip_link(self.arm_root_link, '--arm-root-link')
        if self.robot.link_map.has_key(self.head_root_link) == False:
            self.error_tip_link(self.head_root_link, '--head-root-link')
        if self.robot.link_map.has_key(self.arm_tip_link) == False:
            self.error_tip_link(self.arm_tip_link, '--arm-tip-link')
        if self.robot.link_map.has_key(self.head_tip_link) == False:
            self.error_tip_link(self.head_tip_link, '--head-tip-link')
        if self.robot.link_map.has_key(self.head_camera_frame) == False:
            self.error_tip_link(self.head_camera_frame, '--head-camera-frame')
        if self.robot.joint_map.has_key(self.head_camera_joint) == False:
            self.error_joint(self.head_camera_joint, '--head-camera-joint')

        all_chain = []
        for limb, base_link, end_link in [('arm', self.arm_root_link, self.arm_tip_link), ('head', self.head_root_link, self.head_tip_link)]:
            all_chain.append(self.robot.get_chain(base_link, end_link)[1:])
        for c1,c2 in itertools.product(*all_chain):
            if c1 == c2 :
                rospy.logerr('arm/head chain share same joint ({}), this will cause failure'.format(c1))
                sys.exit()

        for limb, base_link, end_link in [('arm', self.arm_root_link, self.arm_tip_link), ('head', self.head_root_link, self.head_tip_link)]:
            joint_list = filter(lambda x: self.robot.joint_map.has_key(x) and self.robot.joint_map[x].type != 'fixed', [c for c in self.robot.get_chain(base_link, end_link)])
            exec('self.{0}_joint_list = {1}'.format(limb, joint_list)) in locals()

        rospy.loginfo('using following joint for {} chain'.format('arm'))
        rospy.loginfo('... {}'.format(self.arm_joint_list))
        rospy.loginfo('using following joint for {} chain'.format('head'))
        rospy.loginfo('... {}'.format(self.head_joint_list))

        # create robot_calibration directory
        self.dirname_top = self.robot_name+'_calibration'
        self.dirname_capture = self.robot_name+'_calibration/capture_data'
        self.dirname_estimate = self.robot_name+'_calibration/estimate_params'
        self.dirname_results = self.robot_name+'_calibration/view_results'

        try:
            os.makedirs(self.dirname_top)
            os.makedirs(self.dirname_capture)
            os.makedirs(self.dirname_estimate)
            os.makedirs(self.dirname_results)
        except Exception as e:
            rospy.logfatal(e)
            #sys.exit(-1)

        # setup capture_data
        self.write_all_pipelines(args.use_kinect)       # all_pipelines.launch
        self.write_all_viewers(args.use_kinect)         # all_viewers.launch
        self.write_capture_data()                       # capture_data.launch
        self.write_capture_exec()                       # capture_exec.launch
        self.write_hardware_config()                    # hardware_config
        self.write_make_samples()                       # make_samples.py
        # setup estimate_params
        self.write_estimation_config()                  # estimation_config.launch
        self.write_calibrate_sh()                       # calibrate_<robot>.sh
        self.write_estimate_params_config(args.use_kinect)             # free_arms.yaml  free_cameras.yaml  free_cb_locations.yaml  free_torso.yaml  system.yaml
        # setup view_results
        self.write_view_results()                       # scatter_config.yaml  view_scatter.sh


    def error_tip_link(self, link, option):
        rospy.logfatal('could not find valid link name ... {}'.format(link))
        rospy.logfatal('use {0} from one of following names {1}'.format(option, map(lambda x: x.name, self.robot.links)))
        f = open('/tmp/{0}.xml'.format(self.robot_name), 'w+')
        print >>f, self.robot_description
        f.close()
        rospy.logfatal('try \'urdf_to_graphiz /tmp/{}.xml\'.'.format(self.robot_name))
        sys.exit(-1)

    def error_joint(self, joint, option):
        rospy.logfatal('could not find valid joint name ... {}'.format(joint))
        rospy.logfatal('use {0} from one of following names {1}'.format(option, map(lambda x: x.name, self.robot.joints)))
        f = open('/tmp/{0}.xml'.format(self.robot_name), 'w+')
        print >>f, self.robot_description
        f.close()
        rospy.logfatal('try \'urdf_to_graphiz /tmp/{}.xml\'.'.format(self.robot_name))
        sys.exit(-1)

    def write_all_pipelines(self, use_kinect):
        file_name = self.dirname_capture+'/all_pipelines.launch'
        f = open(file_name,'w+')
        f.write("""
<launch>
  <include file="$(find calibration_launch)/capture_data/chain_settler.launch" ns="arm_chain" />
  <include file="$(find calibration_launch)/capture_data/chain_settler.launch" ns="head_chain" />

  <arg name="use_kinect" default="{0}" />
  <group if="$(arg use_kinect)" >
    <include file="$(find calibration_launch)/capture_data/kinect_pipeline.launch" ns="{1}">
      <arg name="image_topic" value="image_rect_color"/> <!-- openni2.launch publishes image_raw and image_rect_color -->
      <arg name="depth_topic" value="/camera/depth/image"/> <!-- this is floar value -->
      <arg name="camera_info_topic" value="camera_info"/>
    </include>
  </group>
  <group unless="$(arg use_kinect)" >
    <include file="$(find calibration_launch)/capture_data/monocam_pipeline.launch" ns="{1}">
      <arg name="image_topic" value="image_rect_color"/> <!-- openni2.launch publishes image_raw and image_rect_color -->
    </include>
  </group>

  <node type="interval_intersection_action"
        pkg="interval_intersection"
        name="interval_intersection"
        output="screen">
    <remap from="head_chain" to="head_chain/settled_interval" />
    <remap from="arm_chain" to="arm_chain/settled_interval" />
    <remap from="head_camera" to="{1}/settled_interval" />
  </node>

</launch>
""".format("true" if use_kinect else "false", self.camera_namespace))
        f.close()

    def write_all_viewers(self, use_kinect):
        file_name = self.dirname_capture+'/all_viewers.launch'
        f = open(file_name,'w+')
        f.write("""
<launch>

  <!-- Hack to create the directory -->
  <param name="mkdir_tmp" command="mkdir -m 777 -p /tmp/{0}_calibration" />
  <param name="mkdir_tmp" command="mkdir -m 777 -p /tmp/{0}_calibration/cb_fail" />

  <param name="{2}/annotated_viewer/filename_format" type="string" value="/tmp/{0}_calibration/cb_fail/cb_{1}_%04i.jpg" />

  <include file="$(find calibration_launch)/capture_data/annotated_viewer.launch"
           ns="{2}" >
   <arg name="image_topic" value="image_rect_annotated" />
  </include>

</launch>
""".format(self.robot_name,"kinect" if use_kinect else "monocam", self.camera_namespace))
        f.close()

    def write_capture_data(self):
        file_name = self.dirname_capture+'/capture_data.launch'
        f = open(file_name,'w+')
        f.write("""
<launch>
  <include file="$(find {0}_calibration)/capture_data/all_viewers.launch"/>
  <include file="$(find {0}_calibration)/capture_data/all_pipelines.launch"/>
  <include file="$(find {0}_calibration)/capture_data/capture_exec.launch"/>
</launch>
""".format(self.robot_name))

    def write_capture_exec(self):
        file_name = self.dirname_capture+'/capture_exec.launch'
        f = open(file_name,'w+')
        f.write("""
<launch>

  <node type="capture_exec.py"
        pkg="calibration_launch"
        name="calibration_exec"
        args="$(find {0}_calibration)/capture_data/samples/ $(find {0}_calibration)/capture_data/hardware_config $(find {0}_calibration)/estimate_params/config/system.yaml"
        output="screen" >
    <remap from="head_camera/camera_info" to="{1}/camera_info"/>
    <remap from="head_camera/image_rect"  to="{1}/image_rect_throttle"/>
    <remap from="head_camera/image"       to="{1}/image_rect_throttle"/>
    <remap from="head_camera/features"    to="{1}/features"/>
  </node>

  <node type="urdf_pub.py" pkg="calibration_launch" name="urdf_pub"/>

  <node type="record" pkg="rosbag" name="robot_measurement_recorder" output="screen"
        args="-O /tmp/{0}_calibration/cal_measurements robot_measurement robot_description" >
    <!-- Hack to create the directory -->
    <param name="mkdir_tmp" command="mkdir -m 777 -p /tmp/{0}_calibration" />
  </node>

</launch>""".format(self.robot_name, self.camera_namespace))
        f.close()

    def write_hardware_config(self):
        config_dir = self.dirname_capture+'/hardware_config/'
        try:
            os.makedirs(config_dir)
        except Exception as e:
            rospy.logfatal(e)

        # cam_config.yaml
        f = open(config_dir+'/cam_config.yaml', 'w+')
        f.write("""
# ----- Microsoft Kinect -----
head_camera: ###FIX
  cb_detector_config:  {0}/cb_detector_config ## FIXME
  led_detector_config: {0}/led_detector
  settler_config:      {0}/monocam_settler_config ## FIXME

  configs:
    small_cb_4x5:
      settler:
        tolerance: 2.00
        ignore_failures: True
        max_step: 3.0
        cache_size: 100
      cb_detector:
        active: True
        num_x: 4
        num_y: 5
        width_scaling: 0.5
        height_scaling: 0.5
        subpixel_window: 4
        subpixel_zero_zone: 1
      led_detector:
        active: False
""".format(self.camera_namespace))     # not sure if this is works for mono camera
        f.close()

        # chain_config.yaml
        f = open(config_dir+'/chain_config.yaml', 'w+')
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            tolerance = 0.002
            f.write('# ----- {} -----\n'.format(limb))
            f.write('{}_chain:\n'.format(limb))
            f.write('  settler_config:  /{}_chain/settler_config\n\n'.format(limb))
            f.write('  configs:\n')
            f.write('    tight_tol:\n')
            f.write('      settler:\n')
            f.write('        joint_names:\n')
            for j in joint_list:
                f.write('        - {}\n'.format(j))
            f.write('        tolerances:\n')
            for j in joint_list:
                f.write('        - {}\n'.format(tolerance))
            f.write('        max_step:   1.0\n')
            f.write('        cache_size: 1500\n\n')
        f.close()

        # controller_config.yaml
        f = open(config_dir+'/controller_config.yaml', 'w+')
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            f.write('{}_controller:\n'.format(limb))
            exec('controller = self.{}_controller'.format(limb))
            rospy.loginfo('Subscribing controller topic is \'{}\''.format(controller))
            f.write('  topic: {0}\n'.format(controller))
            f.write('  joint_names:\n')
            for j in joint_list:
                f.write('  - {}\n'.format(j))
        f.close()

        # laser_config.yaml
        f = open(config_dir+'/laser_config.yaml', 'w+')
        f.write('# laser config')
        f.close()

    def write_make_samples(self):
        samples_dir = self.dirname_capture+'/samples/'
        try:
            os.makedirs(samples_dir+'/arm')
        except Exception as e:
            rospy.logfatal(e)
        f = open(samples_dir+'/arm/config.yaml', 'w+')
        f.write("""
group: "Arm"
prompt: "Please put the checkerboard in the hand (open/close the gripper with the joystick's square/circle buttons)."
finish: "Skipping arm samples"
repeat: False
""")
        f.close()

        f = open(samples_dir+'/../make_samples.py', 'w+')
        os.chmod(samples_dir+'/../make_samples.py', 0744)
        f.write("""#!/usr/bin/env python

# capture samples!!!!1!one

# this script should eventually be replaced by something that finds
# samples automatically

import rospy
from sensor_msgs.msg import JointState

import string, os

header1 = \"\"\"camera_measurements:
- {cam_id: head_camera, config: small_cb_4x5}
joint_commands:
\"\"\"
""")
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            f.write("""
header2_{0} = \"\"\"- controller: {0}_controller
  segments:
  - duration: 2.0
    positions: \"\"\"
""".format(limb))
        f.write("""
header3 = \"\"\"joint_measurements:
""")
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            f.write('- {{chain_id: {0}_chain, config: tight_tol}}\n'.format(limb))
        f.write("""
sample_id: arm_\"\"\"

header4 = \"\"\"target: {chain_id: arm_chain, target_id: small_cb_4x5}\"\"\"

class SampleMaker:
    
    def __init__(self):
        rospy.init_node("make_samples")
        rospy.Subscriber("joint_states", JointState, self.callback)
""")
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            f.write('        self.{}_joints = ['.format(limb))
            for j in joint_list:
                f.write('\"{}\", '.format(j))
            f.write(']\n')
            f.write('        self.{0}_state = [0.0 for joint in self.{0}_joints]\n'.format(limb))
        f.write("""
        self.count = 0

        while not rospy.is_shutdown():
            print "Move arm/head to a new sample position."
            resp = raw_input("press <enter> ")
            if string.upper(resp) == "EXIT":
                break
            else:
                # save a sample:
                count = str(self.count).zfill(4)
                f = open(os.path.dirname(__file__)+"/samples/arm/arm_"+count+".yaml", "w")
                f.write(header1)
""")
        f.write("""                print('saving ... {0}'.format(self.count))\n""")
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            f.write("""                print('  {0}_state: {{0}}'.format(self.{0}_state))\n""".format(limb))
            f.write("""
                f.write(header2_{0})
                print>>f, self.{0}_state
""".format(limb))
        f.write("""
                f.write(header3)
                print>>f, count
                f.write(header4)
            self.count += 1

    def callback(self, msg):
""")
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            f.write("""
        for i in range(len(self.{0}_joints)):
            try:
                idx = msg.name.index(self.{0}_joints[i])
                self.{0}_state[i] = msg.position[idx]
            except: 
                pass
""".format(limb))
        f.write("""

if __name__=="__main__":
    SampleMaker()
""")
        f.close()

    def write_estimation_config(self):
        file_name = self.dirname_estimate+'/estimation_config.launch'
        f = open(file_name,'w+')
        f.write("""
<launch>

  <group ns="calibration_config" clear_params="true">
    <rosparam file="$(find {0}_calibration)/estimate_params/config/system.yaml" command="load" />

    <group ns="cal_steps">

      <group ns="{0} - 00 - Estimating Checkerboard Locations">
        <param name="free_params" textfile="$(find {0}_calibration)/estimate_params/config/free_cb_locations.yaml" />
        <param name="use_cov" type="bool" value="False" />
        <rosparam>
          sensors:
          - arm_chain
          - head_camera
        </rosparam>
        <param name="output_filename" type="string" value="config_0" />
      </group>

      <group ns="{0} - 01 - Adding Camera Locations">
        <param name="free_params" textfile="$(find {0}_calibration)/estimate_params/config/free_cameras.yaml" />
        <param name="use_cov" type="bool" value="True" />
        <rosparam>
          sensors:
          - arm_chain
          - head_camera
        </rosparam>
        <param name="output_filename" type="string" value="system_calibrated" />
      </group>

    </group>

  </group>

</launch>
""".format(self.robot_name))

    def write_calibrate_sh(self):
        file_name = self.dirname_estimate+'/calibrate_'+self.robot_name+'.sh'
        f = open(file_name,'w+')
        os.chmod(file_name, 0744)
        f.write("""#! /bin/bash

if [ -f robot_calibrated.xml ]; then
  echo "./robot_calibrated.xml already exists. Either back up this file or remove it before continuing"
  exit 1
fi

echo "Checking if we can write to ./robot_calibrated.xml..."
touch robot_calibrated.xml
if [ "$?" -ne "0" ]; then
  echo "Not able to write to ./robot_calibrated.xml"
  echo "Make sure you run this script from a directory that for which you have write permissions."
  exit 1
fi
rm robot_calibrated.xml
echo "Success"

roslaunch {0}_calibration estimation_config.launch
rosrun calibration_estimation multi_step_cov_estimator.py /tmp/{0}_calibration/cal_measurements.bag /tmp/{0}_calibration __name:=cal_cov_estimator

est_return_val=$?

if [ "$est_return_val" -ne "0" ]; then
  echo "Estimator exited prematurely with error code [$est_return_val]"
  exit 1
fi

# Make all the temporary files writable
chmod ag+w /tmp/{0}_calibration/*

""".format(self.robot_name))

    def write_estimate_params_config(self, use_kinect=False):
        config_dir = self.dirname_estimate+'/config'
        try:
            os.makedirs(config_dir)
        except Exception as e:
            rospy.logfatal(e)

        # free_arms.yaml
        f = open(config_dir+'/free_arms.yaml', 'w+')
        f.write('\n')
        f.write('  transforms:\n')
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            for j in joint_list:
                f.write('    {0}:     [ 0, 0, 0, {1}, {2}, {3} ]\n'.format(self.robot.joint_map[j].name, self.robot.joint_map[j].axis[0], self.robot.joint_map[j].axis[1], self.robot.joint_map[j].axis[2]))
        f.write('\n')
        f.write('  chains:\n')
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            f.write('    {}_chain:\n'.format(limb))
            f.write('      gearing: {}\n'.format([0]*len(joint_list)))
        f.write("""
  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
    small_cb_4x5:
      spacing_x: 0
      spacing_y: 0

""")
        f.close()

        # free_cameras.yaml
        f = open(config_dir+'/free_cameras.yaml', 'w+')
        f.write("""
  transforms:
    arm_chain_cb: [1, 1, 1, 1, 1, 1]
    {0}: [1, 1, 1, 1, 1, 1]

  chains:
""".format(self.head_camera_joint))
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            f.write('    {}_chain:\n'.format(limb))
            f.write('      gearing: {}\n'.format([0]*len(joint_list)))
        f.write("""
  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
    small_cb_4x5:
      spacing_x: 0
      spacing_y: 0

""")
        f.close()

        # free_cb_locations.yaml
        f = open(config_dir+'/free_cb_locations.yaml', 'w+')
        f.write("""
  transforms:
    arm_chain_cb: [1, 1, 1, 1, 1, 1]

  chains:
""")
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            f.write('    {}_chain:\n'.format(limb))
            f.write('      gearing: {}\n'.format([0]*len(joint_list)))
        f.write("""

  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
    small_cb_4x5:
      spacing_x: 0
      spacing_y: 0

""")

        # free_torso.yaml
        f = open(config_dir+'/free_torso.yaml', 'w+')
        f.write("""
  transforms:
    head_joint: [0, 0, 1, 0, 0, 0]

  chains:
""")
        for limb, joint_list in zip(['arm', 'head'], [self.arm_joint_list, self.head_joint_list]):
            if len(joint_list) == 0:
                continue
            f.write('    {}_chain:\n'.format(limb))
            f.write('      gearing: {}\n'.format([0]*len(joint_list)))
        f.write("""

  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
    small_cb_4x5:
      spacing_x: 0
      spacing_y: 0

""")
        # system.yaml
        f = open(config_dir+'/system.yaml', 'w+')
        f.write('\n')
        f.write('base_link: {0}\n'.format(self.base_link))
        f.write('\n')
        f.write('sensors:\n')
        f.write('\n')
        f.write('  chains:\n')
        if len(self.arm_joint_list) > 0:
            f.write('    arm_chain:\n')
            f.write('      root: {}\n'.format(self.arm_root_link))
            f.write('      tip: {}\n'.format(self.arm_tip_link))
            f.write('      cov:\n')
            f.write('       joint_angles: {}\n'.format([0.01]*len(self.arm_joint_list)))
            f.write('      gearing: {}\n'.format([1.0]*len(self.arm_joint_list)))
        if len(self.head_joint_list) > 0:
            f.write('    head_chain:\n')
            f.write('      root: {}\n'.format(self.head_root_link))
            f.write('      tip: {}\n'.format(self.head_tip_link))
            f.write('      cov:\n')
            f.write('       joint_angles: {}\n'.format([0.01]*len(self.head_joint_list)))
            f.write('      gearing: {}\n'.format([1.0]*len(self.head_joint_list)))
        f.write('\n')
        f.write('  rectified_cams:\n')
        f.write('    head_camera:\n')
        f.write('      chain_id: head_chain  #TODO: get rid of this\n')
        f.write('      frame_id: {}\n'.format(self.head_camera_frame))
        f.write('      baseline_shift: 0.0\n')
        if use_kinect:
            f.write('      baseline_rgbd: 0.075\n')
        else:
            f.write('#      baseline_rgbd: 0.075 ## comment out if we run all_pipelines.launch with use_kinect\n')
        f.write('      f_shift: 0.0\n')
        f.write('      cx_shift: 0.0\n')
        f.write('      cy_shift: 0.0\n')
        f.write('      cov: {u: 0.25, v: 0.25, x: 0.25}\n')
        f.write('\n')
        f.write('  tilting_lasers: {}\n')
        f.write('\n')
        f.write('checkerboards:\n')
        f.write('  small_cb_4x5:\n')
        f.write('    corners_x: 4\n')
        f.write('    corners_y: 5\n')
        f.write('    spacing_x: 0.0245\n')
        f.write('    spacing_y: 0.0245\n')
        f.write('\n')
        f.write('transforms:\n')
        f.write('  arm_chain_cb:   [ .25, 0, 0, pi/2, 0, 0]\n')
        f.write('\n')

    def write_view_results(self):
        file_name = self.dirname_results+'/scatter_config.yaml'
        f = open(file_name, 'w+')
        f.write('- name: "Head Camera: Arm"\n')
        f.write('  3d: arm_chain\n')
        f.write('  cam: head_camera\n')
        f.write('  plot_ops:\n')
        f.write('    color: g\n')
        f.write('    marker: s\n')
        f.close()

        file_name = self.dirname_results+'/view_scatter.sh'
        f = open(file_name,'w+')
        os.chmod(file_name, 0744)
        f.write("""#!/bin/bash
rosrun calibration_estimation error_visualization.py /tmp/{0}_calibration/cal_measurements.bag /tmp/{0}_calibration/ `rospack find {0}_calibration`/view_results/scatter_config.yaml\n""".format(self.robot_name))
        f.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibration setup helper.')
    parser.add_argument('robot_description', type=file, default=False, nargs='?', help='path to urdf file')
    parser.add_argument('--use-kinect', action='store_const', const=True, default=False, help='use kinect for calibraiton')
    parser.add_argument('--base-link', help='base link name')
    parser.add_argument('--arm-root-link', help='arm root link name')
    parser.add_argument('--arm-tip-link', help='arm tip link name')
    parser.add_argument('--head-root-link', help='head root link name')
    parser.add_argument('--head-tip-link', help='head tip link name')
    parser.add_argument('--arm-controller', help='arm controller name', default='arm_controller/command')
    parser.add_argument('--head-controller', help='head controller name', default='head_controller/command')
    parser.add_argument('--head-camera-frame', help='frame name for target camera', default='camera_rgb_optical_frame')
    parser.add_argument('--head-camera-joint', help='joint between head and camera', default='camera_rgb_joint')
    parser.add_argument('--camera-namespace', help='namespace for camera', default='/camera/rgb')
    args = parser.parse_args()

    if args.arm_root_link == None:
        args.arm_root_link = args.base_link
    if args.head_root_link == None:
        args.head_root_link = args.base_link

    rospy.init_node("claibration_setup_helper", anonymous=True)

    write_upload_launch = False
    try:
        if args.robot_description:
            robot_description = args.robot_description.read()
            write_upload_launch = True
        else:
            robot_description = rospy.get_param('robot_description')
    except:
        rospy.logfatal('robot_description not set, exiting')
        sys.exit(-1)

    helper = CalibrationSetupHelper(robot_description, args.base_link, args.arm_root_link, args.arm_tip_link, args.head_root_link, args.head_tip_link, args.arm_controller, args.head_controller, args.head_camera_frame, args.head_camera_joint, args.camera_namespace)

    f = open(helper.dirname_top+'/package.xml', 'w+')
    f.write("""
<package>
  <name>{robot_name}_calibration</name>
  <version>0.2.0</version>
  <description>
    Launch and configuration files for calibrating {robot_name} using the new generic 'calibration' stack.
   THIS FILE IS AUTOMATICALLY GENERATED BY:
   {command}
  </description>
  <author>Calibration Setup Helper</author>
  <maintainer email="kei.okada@gmail.com">Calibration Setup Helper</maintainer>

  <license>BSD</license>
  <url>http://ros.org/wiki/{robot_name}_calibration</url>

  <buildtool_depend>catkin</buildtool_depend>

  <run_depend>calibration_launch</run_depend>
  <run_depend>calibration_estimation</run_depend>
  <run_depend>kdl</run_depend>
  <run_depend>kdl_parser</run_depend>-->
</package>
""".format(robot_name=helper.robot_name, command=' '.join(sys.argv)))
    f.close()

    f = open(helper.dirname_top+'/CMakeLists.txt', 'w+')
    f.write("""
cmake_minimum_required(VERSION 2.8.3)
project({0}_calibration)

find_package(catkin)
catkin_package()

install(DIRECTORY capture_data estimate_params # view_results
  DESTINATION ${{CATKIN_PACKAGE_SHARE_DESTINATION}}
  USE_SOURCE_PERMISSIONS
)

""".format(helper.robot_name))
    f.close()

    if write_upload_launch:
        f = open(helper.dirname_top+'/upload_urdf.launch', 'w+')
        f.write("""
<launch>
    <arg name="model" default="{}" />
    <!-- send maxwell urdf to param server -->
    <param name="robot_description" textfile="$(arg model)" />
</launch>

""".format(args.robot_description.name))
        f.close()
