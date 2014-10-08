/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef FilterInterface_RCARS_HPP_
#define FilterInterface_RCARS_HPP_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <FilterRCARS.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <rcars_detector/TagArray.h>
#include <rcars_detector/TagPoses.h>

#include "ParameterLoader.hpp"

class FilterInterface_RCARS: public FilterRCARS::Filter<3>{
 public:
  /*!
   * Constructor.
   */
  FilterInterface_RCARS(ros::NodeHandle& nh);

  /*!
   * Initialize using a given IMU measurement.
   */
  void initializeFilterWithIMUMeas(const mtPredictionMeas& meas, const double& t);

  /*!
   * Initialize using a given tag measurement.
   */
  void initializeFilterWithTag(const mtUpdateMeas& meas, const double& t);

  /*!
   * Callback for IMU ros messages.
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

  /*!
   * Callback for visual tag ros messages.
   */
  void visionCallback(const rcars_detector::TagArray::ConstPtr& vision_msg);

  /*!
   * Callback for camera info ros messages.
   */
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr cameraInfo_msg);

  /*!
   * Updates the filter state based on the stored measurements and publishes the result
   */
  void updateAndPublish(void);

  /*!
   * Publishes the current tag poses
   */
  void publishTagPoses(void);

 private:
  /*!
   * Time of initialization
   */
  double initializationTime_;
  /*!
   * Flag. True if the filter is initialized.
   */
  bool isInitialized_;
  /*!
   * Flag. True if the camera info is available.
   */
  bool camInfoAvailable_;
  /*!
   * ID of initialization tag, -1 if no intialization tag is required.
   */
  int initTag_;
  /*!
   * Flag. True if the initialization tag was found.
   */
  bool foundInitTag_;
  /*!
   * Time of first observation of initialization tag
   */
  double foundInitTagTime_;


  /*!
   * Ros publishers and subscribers
   */
  ros::Subscriber subImu_;
  ros::Subscriber subTags_;
  ros::Subscriber subCameraInfo_;
  ros::Publisher pubPose_;
  ros::Publisher pubPose2_;
  ros::Publisher pubPose3_;
  ros::Publisher pubPose4_;
  ros::Publisher pubTagPoses_;
  ros::Publisher pubPoseSafe_;
  ros::Publisher pubTwistSafe_;
  ros::Publisher pubTagPosesBody_;
  ros::Publisher pubTagVis_;
  ros::Publisher pubPoseTag1_;
  ros::Publisher pubPoseTag1InvertedRotationTranslation_;
  ros::Publisher pubPoseTag1InvertedRotation_;
};

#endif /* FilterInterface_RCARS_HPP_ */
