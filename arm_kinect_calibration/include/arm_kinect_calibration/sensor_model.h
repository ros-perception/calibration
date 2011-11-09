/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Sachin Chitta */

#include <ros/ros.h>
#include <angles/angles.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

namespace arm_kinect_calibration
{

  /**
   * @class SensorModel
   * @brief A simple geometric sensor model
   */
class SensorModel
{
public:

  /**
   * @brief A default constructor for a sensor model
   * @param sensor_optical_frame_id The optical frame id for the sensor. The Z axis of this frame is assumed to point along the sensor optical axis.
   * @param fov_degrees field of view in degrees
   * @param min_range minimum range of the sensor in meters
   * @param max_range maximum range of the sensor in meters
   */
  SensorModel(const std::string &sensor_optical_frame_id,
              const unsigned int &fov_degrees,
              const double &min_range,
              const double &max_range);

  ~SensorModel(void);

  /**
   * @brief A default constructor for a sensor model
   * @param sensor_optical_frame_id The optical frame id for the sensor. The Z axis of this frame is assumed to point along the sensor optical axis.
   * @param fov_degrees field of view in degrees
   * @param min_range minimum range of the sensor in meters
   * @param max_range maximum range of the sensor in meters
   */
  virtual void initialize(const std::string &sensor_optical_frame_id,
                          const unsigned int &fov_degrees,
                          const double &min_range,
                          const double &max_range);

  std::string getFrameId();

  virtual void getRandomTargetPose(geometry_msgs::PoseStamped &target_pose);

  virtual bool isPointInView(geometry_msgs::Point &target_point,
                             double &angle);

private:

  double generateRandomNumber(const double &min, const double &max);
  std::string sensor_optical_frame_id_;
  double fov_, min_range_, max_range_;
};

}
