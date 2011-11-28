#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************
PKG = 'image_cb_detector'
NAME = 'image_cb_detector_node'
import roslib; roslib.load_manifest(PKG)

import rospy
import cv
import math

from sensor_msgs.msg import Image
from image_cb_detector.msg import ObjectInImage

from cv_bridge import CvBridge, CvBridgeError

class ImageCbDetector:
  def __init__(self, corners_x, corners_y, spacing_x = None, spacing_y = None, width_scaling = 1, height_scaling = 1):
    self.corners_x = corners_x
    self.corners_y = corners_y
    self.spacing_x = spacing_x
    self.spacing_y = spacing_y
    self.width_scaling = width_scaling
    self.height_scaling = height_scaling

  def get_board_corners(self, corners):
    return (corners[0], corners[self.corners_x  - 1], 
        corners[(self.corners_y - 1) * self.corners_x], corners[len(corners) - 1])

  def detect(self, image):
    #resize the image base on the scaling parameters we've been configured with
    scaled_width = int(.5 + image.width * self.width_scaling)
    scaled_height = int(.5 + image.height * self.height_scaling)
    
    #in cvMat its row, col so height comes before width
    image_scaled = cv.CreateMat(scaled_height, scaled_width, cv.GetElemType(image))
    cv.Resize(image, image_scaled, cv.CV_INTER_LINEAR)

    
    found, corners = cv.FindChessboardCorners(image_scaled, (self.corners_x, self.corners_y), cv.CV_CALIB_CB_ADAPTIVE_THRESH)

    if found:
      rospy.logdebug("Found cb")
      board_corners = self.get_board_corners(corners)
      
      #find the perimeter of the checkerboard
      perimeter = 0.0
      for i in range(len(board_corners)):
        next = (i + 1) % 4
        xdiff = board_corners[i][0] - board_corners[next][0]
        ydiff = board_corners[i][1] - board_corners[next][1]
        perimeter += math.sqrt(xdiff * xdiff + ydiff * ydiff)

      #estimate the square size in pixels
      square_size = perimeter / ((self.corners_x - 1 + self.corners_y - 1) * 2)
      radius = int(square_size * 0.5 + 0.5)

      corners = cv.FindCornerSubPix(image_scaled, corners, (radius, radius), (-1, -1), (cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 30, 0.1))
      #cv.DrawChessboardCorners(image_scaled, (self.corners_x, self.corners_y), corners, 1)
      #cv.NamedWindow("image_scaled")
      #cv.ShowImage("image_scaled", image_scaled)
      #cv.WaitKey()

      object_points = None

      #we'll also generate the object points if they've been requested
      if self.spacing_x != None and self.spacing_y != None:
        object_points = [None] * (self.corners_x * self.corners_y)

        for i in range(self.corners_y):
          for j in range(self.corners_x):
            object_points[i * self.corners_x + j]  = (j * self.spacing_x, i * self.spacing_y)

      return (corners, object_points)

    else:
      rospy.logdebug("Didn't find checkerboard")
      return (None, None)

class ImageCbDetectorNode:
  def __init__(self):
    corners_x = rospy.get_param('~corners_x', 6)
    corners_y = rospy.get_param('~corners_y', 6)
    spacing_x = rospy.get_param('~spacing_x', None)
    spacing_y = rospy.get_param('~spacing_y', None)
    width_scaling = rospy.get_param('~width_scaling', 1)
    height_scaling = rospy.get_param('~height_scaling', 1)

    self.im_cb_detector = ImageCbDetector(corners_x, corners_y, spacing_x, 
        spacing_y, width_scaling, height_scaling)

    rospy.Subscriber("image_stream", Image, self.callback)
    self.corner_pub = rospy.Publisher("corners_in_image", ObjectInImage)
    self.bridge = CvBridge()

  def callback(self, ros_image):
    #we need to convert the ros image to an opencv image
    try:
      image = self.bridge.imgmsg_to_cv(ros_image, "mono8")
    except CvBridgeError, e:
      rospy.logerror("Error importing image %s" % e)
      return

    corners, model = self.im_cb_detector.detect(image)

    #now we need to publish them out over ros
    obj_msg = ObjectInImage()
    obj_msg.header = ros_image.header

    if corners != None:
      obj_msg.set_image_points_size(len(corners))
      for i in range(len(corners)):
        obj_msg.image_points[i].x = corners[i][0]
        obj_msg.image_points[i].y = corners[i][1]

    if model != None:
      obj_msg.set_model_points_size(len(model))
      for i in range(len(corners)):
        obj_msg.model_points[i].x = model[i][0]
        obj_msg.model_points[i].y = model[i][1]

    self.corner_pub.publish(obj_msg)

def cb_detector_main(argv=None):
  rospy.init_node(NAME, anonymous=False)
  cb_detector = ImageCbDetectorNode()
  rospy.spin()

if __name__ == '__main__':
  cb_detector_main()

