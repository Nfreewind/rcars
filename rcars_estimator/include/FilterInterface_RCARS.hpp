/*
* Copyright (c) 2014, Michael Neunert & Michael Blösch
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the ETH Zurich nor the
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
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <config.hpp>
#include <FilterRCARS.hpp>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <rcars_detector/TagArray.h>
#include <rcars_detector/TagPoses.h>

class FilterInterface_RCARS: public rcars::FilterRCARS<nDynamicTags,nHybridTags>{
 public:
  /*!
   * Typedefs and using-declarations
   */
  typedef rcars::PredictionMeas mtPredictionMeas;
  typedef typename rcars::TagUpdate<mtFilterState>::mtMeas mtUpdateMeas;

  static constexpr int nTags_ =  mtState::nTags_;

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
   * Callback function for service that saves the current workspace
   */
  bool saveWorkspaceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!
   * Callback for service that resets the filter
   */
  bool resetServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

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

  /*!
   * Safe workspace to config
   */
  void saveWorkspace();

 private:
  /*!
   * Loads workspace from config
   */
  void loadWorkspace();


  /*!
    * Boolean if vision data is available
    */
  bool visionDataAvailable_;

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
   * ID of reference tag for workspace alignment, -1 if no reference tag is configured.
   */
  int referenceTagId_;

  /*!
   * Counter how many times a tag has to be seen until it gets added to the calibrated workspace.
   */
  int calibrationViewCountThreshold_;

  /*!
   * Map from tagId to tag type
   */
  std::map<int, rcars::TagType> tagType_;

  /*!
   * Counts how often a tag has been seen
   */
  std::map<int, unsigned int> tagViewCount_;

  /*!
   * Counts how often a tag has been seen while a second one was seen at the same time
   */
  std::map<int, unsigned int> tagViewOverlapCount_;

  /*!
   * Map from tagId to tag position for calibratedTags
   */
  std::map<int, Eigen::Vector3d, std::less<int>,
           Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3d> > > IrIT_;

  /*!
   * Map from tagId to tag orientation for calibratedTags
   */
  std::map<int, rot::RotationQuaternionPD, std::less<int>,
           Eigen::aligned_allocator<std::pair<const int, rot::RotationQuaternionPD> > > qTI_;

  /*!
   * Orientation of workspace with respect to reference tag
   */
  rot::RotationQuaternionPD qTW_;

  /*!
   * Location of reference tag in workspace
   */
  Eigen::Vector3d WrWT_;

  /*!
   * Flag if workspace gets overwritten (or just updated)
   */
  bool overwriteWorkspace_;

  /*!
   * Ros publishers and subscribers
   */
  ros::NodeHandle& nh_;

  ros::Subscriber subImu_;
  ros::Subscriber subTags_;
  ros::Subscriber subCameraInfo_;
  ros::Publisher pubPose_;
  ros::Publisher pubTagArrayCameraFrame_;
  ros::Publisher pubTagArrayInertialFrame_;
  ros::Publisher pubPoseSafe_;
  ros::Publisher pubTwistSafe_;
  ros::Publisher pubExtrinsics_;

  ros::ServiceServer resetService_;
  ros::ServiceServer saveWorkspaceService_;
};

#endif /* FilterInterface_RCARS_HPP_ */
