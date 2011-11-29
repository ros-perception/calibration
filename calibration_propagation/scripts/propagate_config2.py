#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2_calibration_propagation')
import rospy
import pr2_calibration_propagation.update_urdf as update_urdf

import math
import pdb
import sys
# from ros import rosrecord
import rosbag



try:
    import yaml
except ImportError, e:
    print >> sys.stderr, "Cannot import yaml. Please make sure the pyyaml system dependency is installed"
    raise e

if __name__ == '__main__':

    #rospy.init_node("propagate_config")

    if len(rospy.myargv()) < 5:
        print "Usage: ./propagate_config [initial.yaml] [calibrated.yaml] [cal_measurements.bag] [cal_output.xml]"
        sys.exit(0)

    #filenames
    initial_yaml_filename    = rospy.myargv()[1]
    calibrated_yaml_filename = rospy.myargv()[2]
    measurement_filename     = rospy.myargv()[3]
    output_filename          = rospy.myargv()[4]

    bag = rosbag.Bag(measurement_filename)
    xml_in = None
    for topic, msg, t in bag.read_messages():
        if topic == "robot_description" or topic == "/robot_description":
            xml_in = msg.data
    if xml_in is None:
        print "Error: Could not find URDF in bagfile. Make sure topic 'robot_description' exists"
        sys.exit(-1)
    bag.close()

    #read in files
    system_default = yaml.load(file(initial_yaml_filename, 'r'))
    system_calibrated = yaml.load(file(calibrated_yaml_filename, 'r'))

    xml_out = update_urdf.update_urdf(system_default, system_calibrated, xml_in)

    outfile = open(output_filename, 'w')
    outfile.write(xml_out)
    outfile.close()
