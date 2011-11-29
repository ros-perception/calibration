#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2_calibration_propagation')
import tf.transformations as transformations
import rospy

import math
import pdb
import sys

try:
    import yaml
except ImportError, e:
    print >> sys.stderr, "Cannot import yaml. Please make sure the pyyaml system dependency is installed"
    raise e

#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


#return 1 if value1 and value2 are within eps of each other, 0 otherwise
def epsEq(value1, value2, eps = 1e-10):
    if math.fabs(value1-value2) <= eps:
        return 1
    return 0


#convert a float/int/string containing 'pi' to just float
def mixed_to_float(mixed):
    pi = math.pi
    if type(mixed) == str:
        try:
            val = eval(mixed)
        except:
            print >> sys.stderr, "bad value:", mixed, "substituting zero!!\n\n"
            val = 0.
    else:
        val = float(mixed)
    return val


#calculate calibration offsets (whicharm = 'left' or 'right')
def find_dh_param_offsets(chain_name):
    offsets = []
    for (default_params, calib_params) in zip(system_default['dh_chains'][chain_name]['dh'], system_calibrated['dh_chains'][chain_name]['dh']):
        #print "default_params[0]:", default_params[0], "calib_params[0]:", calib_params[0]
        diff = mixed_to_float(calib_params[0]) - mixed_to_float(default_params[0])
        if epsEq(diff, 0):
            diff = 0.
        offsets.append(diff)

    return offsets



#convert from rotation-axis-with-angle-as-magnitude representation to Euler RPY
def angle_axis_to_RPY(vec):
    angle = math.sqrt(sum([vec[i]**2 for i in range(3)]))
    hsa = math.sin(angle/2.)
    if epsEq(angle, 0):
        return (0.,0.,0.)
    quat = [vec[0]/angle*hsa, vec[1]/angle*hsa, vec[2]/angle*hsa, math.cos(angle/2.)]
    rpy = quat_to_rpy(quat)
    return rpy

def rpy_to_quat(rpy):
    return transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], 'sxyz')

def quat_to_rpy(q):
    rpy = transformations.euler_from_quaternion(q, 'sxyz')
    return rpy

def parse_rpy(line):
    return [float(x) for x in line.split("rpy=\"")[1].split("\"")[0].split()]

def parse_xyz(line):
    return [float(x) for x in line.split("xyz=\"")[1].split("\"")[0].split()]

if __name__ == '__main__':

    rospy.init_node("propagate_config")

    if len(rospy.myargv()) < 5:
        print "Usage: ./propagate_config [initial.yaml] [calibrated.yaml] [initial.xml] [cal_output.xml]"
        sys.exit(0)

    #filenames
    initial_yaml_filename    = rospy.myargv()[1]
    calibrated_yaml_filename = rospy.myargv()[2]
    robot_xml_filename       = rospy.myargv()[3]
    output_filename          = rospy.myargv()[4]

    #read in files
    system_default = yaml.load(file(initial_yaml_filename, 'r'))
    system_calibrated = yaml.load(file(calibrated_yaml_filename, 'r'))


    #find dh_param offsets for all requested dh chains
    dh_offsets = {"right_arm_chain":[],
                  "left_arm_chain":[],
                  "head_chain":[]}

    dh_joint_names = {"right_arm_chain" : ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'],
                      "left_arm_chain"  : ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'],
                      "head_chain"      : ['head_pan_joint', 'head_tilt_joint'] }

    # Check that the chains are in fact in the yaml system config
    chains_to_remove = [x for x in dh_offsets.keys() if x not in system_default['dh_chains'].keys()];
    print "Need to ignore the following chains:", chains_to_remove
    for chain_to_remove in chains_to_remove:
      del dh_offsets[chain_to_remove]

    print "Computing All dh chain offsets"
    for chain_name in dh_offsets.keys():
        dh_offsets[chain_name] = find_dh_param_offsets(chain_name)
        print "%s offsets:" % chain_name, pplist(dh_offsets[chain_name])

    # Need to be able to lookup the joint offset for each joint
    joint_offsets_list = []
    for chain_name in dh_offsets.keys():
        joint_offsets_list.extend(zip(dh_joint_names[chain_name], dh_offsets[chain_name]))
    joint_offsets = dict(joint_offsets_list)
    foundjointnames = []

    #convert transforms to rpy
    transformdict = dict()
    for(name, rotvect) in system_calibrated['transforms'].iteritems():
        floatvect = [mixed_to_float(x) for x in rotvect]
        #print name, pplist(floatvect), angle_axis_to_RPY(floatvect[3:6])
        transformdict[name] = tuple(list(angle_axis_to_RPY(floatvect[3:6])) + floatvect[0:3])

    # Hack in transforms for tilting laser
    floatvect = [mixed_to_float(x) for x in system_calibrated['tilting_lasers']['tilt_laser']['before_joint'] ]
    transformdict['laser_tilt_mount_joint'] = tuple(list(angle_axis_to_RPY(floatvect[3:6])) + floatvect[0:3])

    #print "Floatvec: ", floatvec
    #print "tuple: ", transformdict['laser_tilt_mount_joint']
    #import code; code.interact(local=locals())
    #assert(False)

    floatvect = [mixed_to_float(x) for x in system_calibrated['tilting_lasers']['tilt_laser']['after_joint'] ]
    transformdict['laser_tilt_joint'] = tuple(list(angle_axis_to_RPY(floatvect[3:6])) + floatvect[0:3])

    foundtransforms = []

    #go through xml, converting values as they come up
    xmllines = file(robot_xml_filename, 'r').readlines()
    outfile = file(output_filename, 'w')
    lineind = 0
    while(lineind < len(xmllines)):
        looking_for_calibration_pos = 0
        looking_for_origin = 0
        #check for dh_param_search_strings
        for joint_name in joint_offsets.keys():
            search_string = joint_name + "\" type="
            if search_string in xmllines[lineind]:
                #found one: start looking for the calibration reference position
                print "Found joint: ", joint_name
                foundjointnames.append(joint_name)
                looking_for_calibration_pos = 1
                break

        #check for transform joint name
        for (transform_name, rpy_trans) in transformdict.iteritems():
            if transform_name+"\" type=" in xmllines[lineind]:

                #found one: start looking for the origin
                print "found transform for", transform_name
                foundtransforms.append(transform_name)
                looking_for_origin = 1
                break

        if looking_for_calibration_pos or looking_for_origin:
            # Extract the entire joint tag
            searchind = 0
            joint_lines = []
            while(searchind < 50):
                joint_lines.append(xmllines[searchind+lineind])
                if "</joint>" in xmllines[searchind+lineind]:
                    lineind += searchind
                    break
                searchind += 1
            else:
                print >> sys.stderr, "looked for 50 lines and didn't find what we were looking for.  Is the joint description more than 50 lines?  Go change this script, if so.\n\n\n"

            print "Extracted joint:"
            print "------"
            for joint_line_ind, cur_line in zip(range(len(joint_lines)), joint_lines):
                print "% 2u)" % joint_line_ind, cur_line
            print "------"

            # Now that we have all the lines for the joint, figure out which parts of useful
            axis_xyz = None
            origin_rpy = None
            origin_xyz = None
            origin_ind = None

            # Calibration reference position line index and value
            calibration_ind = None
            calibration_ref_pos = None
            calibration_rising = None
            calibration_falling = None


            for joint_line_ind in range(len(joint_lines)):
                if "<axis " in joint_lines[joint_line_ind]:
                    print "  Found axis tag at line %u" % joint_line_ind
                    axis_xyz = parse_xyz(joint_lines[joint_line_ind])
                    print "    xyz: %s" % axis_xyz
                if "<origin " in joint_lines[joint_line_ind]:
                    print "  Found origin tag at line %u" % joint_line_ind
                    origin_ind = joint_line_ind

                    # Get floats for important fields
                    origin_xyz = parse_xyz(joint_lines[joint_line_ind])
                    origin_rpy = parse_rpy(joint_lines[joint_line_ind])
                    print "    xyz: %s" % origin_xyz
                    print "    rpy: %s" % origin_rpy
                if "<calibration " in joint_lines[joint_line_ind]:
                    print "  Found calibration tag at line %u" % joint_line_ind
                    calibration_ind = joint_line_ind
                    if "reference_position" in joint_lines[calibration_ind]:
                        calibration_ref_pos = float(joint_lines[calibration_ind].split("reference_position=\"")[1].split("\"")[0])
                        print "    reference position: %f" % calibration_ref_pos
                    if "rising" in joint_lines[calibration_ind]:
                        calibration_rising = float(joint_lines[calibration_ind].split("rising=\"")[1].split("\"")[0])
                        print "    rising edge: %f" % calibration_rising
                    if "falling" in joint_lines[calibration_ind]:
                        calibration_falling = float(joint_lines[calibration_ind].split("falling=\"")[1].split("\"")[0])
                        print "    falling edge: %f" % calibration_falling

                    

            # Do the update for this joint
            if looking_for_origin:
                print "  Doing a transform update"
                tag_split = joint_lines[origin_ind].split('/>')
                if len(tag_split) == 1:
                    oldlineending = ""
                else:
                    oldlineending = tag_split[1]
                newline = "\t<origin rpy=\"%.10f %.10f %.10f\" xyz=\"%.10f %.10f %.10f\"/>"%rpy_trans + oldlineending
                print "    transform old line:", joint_lines[origin_ind].strip()
                print "    transform new line:", newline.strip()
                joint_lines[origin_ind] = newline
            if looking_for_calibration_pos:
                print "  Doing a calibration position update"
                print "    Shifting by: %f" % joint_offsets[joint_name]

                # Do the text stuff
                tag_split = joint_lines[calibration_ind].split('/>')
                if len(tag_split) == 1:
                    oldlineending = ""
                else:
                    oldlineending = "/>".join(tag_split[1:])

                ref_pos_text = ""
                rising_text = ""
                falling_text = ""

                if calibration_ref_pos is not None:
                    ref_pos_text = "reference_position=\"%.10f\"" % (calibration_ref_pos + joint_offsets[joint_name])

                if calibration_rising is not None:
                    rising_text = "rising=\"%.10f\"" % (calibration_rising + joint_offsets[joint_name])

                if calibration_falling is not None:
                    falling_text = "falling=\"%.10f\"" % (calibration_falling + joint_offsets[joint_name])

                newline = "\t<calibration %s %s %s/>" % (ref_pos_text, rising_text, falling_text) + oldlineending
                print "    calibration old line:", joint_lines[calibration_ind].strip()
                print "    calibration new line:", newline.strip()
                joint_lines[calibration_ind] = newline

            for cur_line in joint_lines:
                outfile.write(cur_line)

        else:
            #just pass the line on unchanged
            outfile.write(xmllines[lineind])
        lineind += 1

    outfile.close()

    #check to see that we found everything we wanted
    notfoundtransforms = transformdict.keys()
    for transform in foundtransforms:
        try:
            notfoundtransforms.remove(transform)
        except:
            print >> sys.stderr, "transform", transform, "found and changed twice, oops!!!\n\n"
    print "transforms never found:", notfoundtransforms

    notfoundjointnames = joint_offsets.keys()
    for jointname in foundjointnames:
        try:
            notfoundjointnames.remove(jointname)
        except:
            print >> sys.stderr, "jointname", jointname, "found and changed twice, oops!!!\n\n"
    print "jointnames never found:", notfoundjointnames
