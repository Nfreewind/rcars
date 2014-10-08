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

#include "FilterInterface_RCARS.hpp"

FilterInterface_RCARS::FilterInterface_RCARS(ros::NodeHandle& nh) {
  LoadParameters("parametersRCARS.info", this);

  // Outlier detection currently disabled
  // enableOutlierDetection();
  disableOutlierDetection();

  // Reset the filter
  reset();

  // Setup subscribers and publishers
  subImu_ = nh.subscribe("/imu0", 1000, &FilterInterface_RCARS::imuCallback,this);
  subTags_ = nh.subscribe("/rcars_detected_tags", 10, &FilterInterface_RCARS::visionCallback,this);
  subCameraInfo_ = nh.subscribe("/cam0/camera_info", 1, &FilterInterface_RCARS::cameraInfoCallback,this);
  pubPose_ = nh.advertise<geometry_msgs::PoseStamped>("filterPose", 1000);
  pubTagPoses_ = nh.advertise<rcars_detector::TagPoses>("tagPoses", 1000);
  pubTagPosesBody_ = nh.advertise<rcars_detector::TagPoses>("tagPosesBody",1000);
  pubTagVis_ = nh.advertise<geometry_msgs::PoseArray>("tagPosesVis",1000);
  pubPoseSafe_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("filterPoseSafe", 1000);
  pubTwistSafe_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("filterTwistSafe", 1000);
  pubPoseTag1_= nh.advertise<geometry_msgs::PoseStamped>("tag1fromBodyEstimated", 1000);
  pubPoseTag1InvertedRotation_= nh.advertise<geometry_msgs::PoseStamped>("tag1fromBodyEstimatedInvertedRotation", 1000);
  pubPoseTag1InvertedRotationTranslation_= nh.advertise<geometry_msgs::PoseStamped>("tag1fromBodyEstimatedInvertedRotationTranslation", 1000);

  // Initialize remaining filter variables
  initializationTime_ = 0;
  isInitialized_ = false;
  camInfoAvailable_ = false;
  initTag_ = -1;
  foundInitTag_ = false;
  foundInitTagTime_ = 0;
}

void FilterInterface_RCARS::initializeFilterWithIMUMeas(const mtPredictionMeas& meas, const double& t) {
  // Reset the filter using the provided accelerometer measurement
  resetWithAccelerometer(meas.acc());
  resetTime(t);
  clean(t);
  isInitialized_ = true;
}

void FilterInterface_RCARS::initializeFilterWithTag(const mtUpdateMeas& meas, const double& t) {
  // Search the initialization tag
  for(unsigned int i=0;i<nTags_;i++){
    if(meas.tagId_[i]==initTag_){
      // Extract relative position and attitude estimate of initialization tag
      Eigen::Vector3d VrVT = meas.tagPos(i);
      rot::RotationQuaternionPD qTV = meas.tagAtt(i);

      // Reset the filter useing these relative measurements
      resetWithTagPose(VrVT,qTV,initTag_);
      resetTime(t);
      clean(t);
      isInitialized_ = true;
    }
  }
}

void FilterInterface_RCARS::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  // Create and fill prediction measurement using the IMU data
  mtPredictionMeas predictionMeas;
  predictionMeas.acc() = Eigen::Vector3d(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z);
  predictionMeas.gyr() = Eigen::Vector3d(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);

  // Check if initialization can be performed (requires the availability of tag measurements)
  if(!isInitialized_ && !updateMeasMap_.empty()) {
    if(initTag_<0){ // If no initialization tag is required initialize with the IMU measurement
      initializationTime_ = imu_msg->header.stamp.toSec();
      initializeFilterWithIMUMeas(predictionMeas,initializationTime_);
    } else if(foundInitTag_){ // If an initialization tag is required and it has been observed, then use the tag for initialization
      initializationTime_ = foundInitTagTime_;
      initializeFilterWithTag(updateMeasMap_[foundInitTagTime_],initializationTime_);
    }
    return;
  } else if(isInitialized_){ // If the filter is initialized add the IMU measurement to the filter and update
    addPredictionMeas(predictionMeas,imu_msg->header.stamp.toSec());
    updateAndPublish();
  }
}

void FilterInterface_RCARS::visionCallback(const rcars_detector::TagArray::ConstPtr& vision_msg) {
  // Return if the camera info is not yet available
  if(!camInfoAvailable_) { return; }

  // If the measurent lies in the past ignore it
  if(isInitialized_ && vision_msg->header.stamp.toSec() <= initializationTime_) { return; }

  // If the filter is not yet initialized try to find the initialization tag if it is required
  if(!isInitialized_ && !foundInitTag_ && initTag_>=0){
    for(unsigned int i=0;i<nTags_;i++){
      if(vision_msg->tags[i].id==initTag_){
        std::cout << "Found init tag!" << std::endl;
        foundInitTag_ = true;
        foundInitTagTime_ = vision_msg->header.stamp.toSec();
      }
    }
  }

  // Create and fill the update measurement
  mtUpdateMeas updateMeas;
  // Determine the number of tags to read out
  size_t tagMax = vision_msg->tags.size();
  if (nTags_ < tagMax) tagMax = nTags_;
  // Read out the tagMax first tags from the current TagArray measurement
  for (size_t i=0; i<tagMax; i++){ // TODO: Prefer tags that are already in the filter
    // Copy the tag index
    updateMeas.tagId_(i) = vision_msg->tags[i].id;
    // Copy the corner measurements
    for (size_t j=0; j<4; j++){
      updateMeas.cor(i,2*j) = vision_msg->tags[i].corners[j].x;
      updateMeas.cor(i,2*j+1) = vision_msg->tags[i].corners[j].y;
    }
    // Copy the estimated pose
    const geometry_msgs::Pose& pose = vision_msg->tags[i].pose;
    updateMeas.tagPos(i) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    updateMeas.tagAtt(i) = rot::RotationQuaternionPD(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  }

  // Add the update measurement and update
  addUpdateMeas(updateMeas, vision_msg->header.stamp.toSec());
  updateAndPublish();
}

void FilterInterface_RCARS::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr cameraInfo_msg){
  // Readout the camera matrix from the CameraInfo message
	for (size_t i=0; i<this->CameraMatrix_.RowsAtCompileTime; i++){
		for (size_t j=0; j<this->CameraMatrix_.ColsAtCompileTime; j++){
			CameraMatrix_(i,j) = cameraInfo_msg->P[i*4 + j];
		}
	}

	// Set the camInfoAvailable_ to true
	if (!camInfoAvailable_){
		ROS_INFO("Received camera info. Will now publish pose estimates for tags as well.");
		camInfoAvailable_ = true;
	}
}

void FilterInterface_RCARS::updateAndPublish(void){
  // TODO(neunertm): clean up
  // Only update if the filter is initialized
  if (isInitialized_) {
    // Store the current time of the filter
    double t = stateSafe_.t_;
    // Do a safe update, i.e., to the prediction or update measurement which lies more back in time
    updateSafe();
    // Check if something has changed, if yes publish filter state
    if(stateSafe_.t_>t){
      // get the estimated position and attitude
      Eigen::Vector3d pos = get_IrIB_safe();
      rot::RotationQuaternionPD quat = get_qBI_safe();

      // Publish the corresponding tf
      static tf::TransformBroadcaster tb_ekf_safe;
      tf::StampedTransform tf_transform_ekf;
      tf_transform_ekf.frame_id_ = "world";
      tf_transform_ekf.child_frame_id_ = "ekf_safe";
      tf_transform_ekf.stamp_ = ros::Time(stateSafe_.t_);
      tf::Transform tf;
      tf.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
      tf.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
      tf_transform_ekf.setData(tf);
      tb_ekf_safe.sendTransform(tf_transform_ekf);

      // Publish the pose with timestamp
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time(stateSafe_.t_);
      pose.header.frame_id = "world";
      pose.pose.position.x = pos(0);
      pose.pose.position.y = pos(1);
      pose.pose.position.z = pos(2);
      pose.pose.orientation.x = quat.x();
      pose.pose.orientation.y = quat.y();
      pose.pose.orientation.z = quat.z();
      pose.pose.orientation.w = quat.w();
      pubPose_.publish(pose);

      // Publish the tag pose 0
      geometry_msgs::PoseStamped pose2;
      pose2.header.stamp = ros::Time(stateSafe_.t_);
      pose2.header.frame_id = "world";
      pose2.pose.position.x = stateSafe_.tagPos(0)(0);
      pose2.pose.position.y = stateSafe_.tagPos(0)(1);
      pose2.pose.position.z = stateSafe_.tagPos(0)(2);
      pose2.pose.orientation.x = stateSafe_.tagAtt(0).x();
      pose2.pose.orientation.y = stateSafe_.tagAtt(0).y();
      pose2.pose.orientation.z = stateSafe_.tagAtt(0).z();
      pose2.pose.orientation.w = stateSafe_.tagAtt(0).w();
      pubPose2_.publish(pose2);

      // Publish the tag pose 1
      geometry_msgs::PoseStamped pose3;
      pose3.header.stamp = ros::Time(stateSafe_.t_);
      pose3.header.frame_id = "world";
      pose3.pose.position.x = stateSafe_.tagPos(1)(0);
      pose3.pose.position.y = stateSafe_.tagPos(1)(1);
      pose3.pose.position.z = stateSafe_.tagPos(1)(2);
      pose3.pose.orientation.x = stateSafe_.tagAtt(1).x();
      pose3.pose.orientation.y = stateSafe_.tagAtt(1).y();
      pose3.pose.orientation.z = stateSafe_.tagAtt(1).z();
      pose3.pose.orientation.w = stateSafe_.tagAtt(1).w();
      pubPose3_.publish(pose3);

      // Publish the tag pose 2
      geometry_msgs::PoseStamped pose4;
      pose4.header.stamp = ros::Time(stateSafe_.t_);
      pose4.header.frame_id = "world";
      pose4.pose.position.x = stateSafe_.tagPos(2)(0);
      pose4.pose.position.y = stateSafe_.tagPos(2)(1);
      pose4.pose.position.z = stateSafe_.tagPos(2)(2);
      pose4.pose.orientation.x = stateSafe_.tagAtt(2).x();
      pose4.pose.orientation.y = stateSafe_.tagAtt(2).y();
      pose4.pose.orientation.z = stateSafe_.tagAtt(2).z();
      pose4.pose.orientation.w = stateSafe_.tagAtt(2).w();
      pubPose4_.publish(pose4);

      // Publish further tag poses
      publishTagPoses();

	  //tag0 -> tag1
	  // qIT1.inverse() * qIT2
	  rot::RotationQuaternionPD qIT1 = stateSafe_.tagAtt(0);
	  rot::RotationQuaternionPD qIT2 = stateSafe_.tagAtt(1);
	  rot::RotationQuaternionPD qT1T2 = qIT1.inverted() * qIT2;
	  //rot::RotationQuaternionPD qT1T2_ref(0.856, 0.029, -0.357, 0.372);
	  //rot::RotationQuaternionPD qT1T2_ref(0.372, 0.856, 0.029, -0.357);

	  if(stateSafe_.tagId_(1) == 1)
	  {
		  std::cout<<stateSafe_.t_<<","<<qIT1.getDisparityAngle(qIT2)<<";"<<std::endl;
	  }


      // Get pose/twist and publish
      Eigen::Vector3d IrIB;
      rot::RotationQuaternionPD qIB;
      Eigen::Vector3d BvB;
      Eigen::Vector3d BwB;
      getOutput(stateSafe_,IrIB,qIB,BvB,BwB);
      Eigen::Matrix<double,12,12> Cov;
      Cov = getOutputCovariance(stateSafe_,stateSafeP_);

      geometry_msgs::PoseWithCovarianceStamped msg;
      msg.pose.pose.position.x = IrIB(0);
      msg.pose.pose.position.y = IrIB(1);
      msg.pose.pose.position.z = IrIB(2);
      msg.pose.pose.orientation.w = qIB.w();
      msg.pose.pose.orientation.x = qIB.x();
      msg.pose.pose.orientation.y = qIB.y();
      msg.pose.pose.orientation.z = qIB.z();
      unsigned int indexArray[6] = {0,1,2,3,4,5};
      for(unsigned int i=0;i<6;i++){
        for(unsigned int j=0;j<6;j++){
          msg.pose.covariance[6*i+j] = Cov(indexArray[i],indexArray[j]);
        }
      }
      msg.header.stamp = ros::Time(stateSafe_.t_);
      pubPoseSafe_.publish(msg);
    }
  }
}

void FilterInterface_RCARS::publishTagPoses(void)
{
	rcars_detector::TagPoses tagPosesMsg;
	tagPosesMsg.header.stamp = ros::Time(stateSafe_.t_);
	tagPosesMsg.header.frame_id = "world";
	tagPosesMsg.tagIds.resize(nTags_);
	tagPosesMsg.poses.resize(nTags_);

	rcars_detector::TagPoses tagPosesBodyMsg;
	tagPosesBodyMsg.header.stamp = ros::Time(stateSafe_.t_);
	tagPosesBodyMsg.header.frame_id = "world";
	tagPosesBodyMsg.tagIds.resize(nTags_);
	tagPosesBodyMsg.poses.resize(nTags_);


	Eigen::Vector3d IrIB;
	rot::RotationQuaternionPD qIB;
	Eigen::Vector3d BvB;
	Eigen::Vector3d BwB;
	getOutput(stateSafe_,IrIB,qIB,BvB,BwB);

	for(unsigned int i=0;i<nTags_;i++){
		tagPosesMsg.tagIds[i] = stateSafe_.tagId_(i);
		tagPosesBodyMsg.tagIds[i] = stateSafe_.tagId_(i);

	    tagPosesMsg.poses[i].position.x = stateSafe_.tagPos(i)(0);
	    tagPosesMsg.poses[i].position.y = stateSafe_.tagPos(i)(1);
	    tagPosesMsg.poses[i].position.z = stateSafe_.tagPos(i)(2);
	    tagPosesMsg.poses[i].orientation.x = stateSafe_.tagAtt(i).x();
	    tagPosesMsg.poses[i].orientation.y = stateSafe_.tagAtt(i).y();
	    tagPosesMsg.poses[i].orientation.z = stateSafe_.tagAtt(i).z();
	    tagPosesMsg.poses[i].orientation.w = stateSafe_.tagAtt(i).w();

	    Eigen::Vector3d IrBT = stateSafe_.tagPos(i) - IrIB;
	    Eigen::Vector3d BrBT = qIB.inverted().rotate(IrBT);

	    tagPosesBodyMsg.poses[i].position.x = BrBT(0);
	    tagPosesBodyMsg.poses[i].position.y = BrBT(1);
	    tagPosesBodyMsg.poses[i].position.z = BrBT(2);

	    rot::RotationQuaternionPD qBT = qIB.inverted() * stateSafe_.tagAtt(i);

	    tagPosesBodyMsg.poses[i].orientation.x = qBT.x();
	    tagPosesBodyMsg.poses[i].orientation.y = qBT.y();
	    tagPosesBodyMsg.poses[i].orientation.z = qBT.z();
	    tagPosesBodyMsg.poses[i].orientation.w = qBT.w();

	    if(stateSafe_.tagId_(i) == 1)
	    {
	    	geometry_msgs::PoseStamped poseTag1Body;
			poseTag1Body.header.stamp = ros::Time(stateSafe_.t_);
			poseTag1Body.header.frame_id = "world";
			poseTag1Body.pose = tagPosesBodyMsg.poses[i];
			pubPoseTag1_.publish(poseTag1Body);


			geometry_msgs::PoseStamped pubPoseTag1InvertedRotationMsg;
			pubPoseTag1InvertedRotationMsg.header.stamp = ros::Time(stateSafe_.t_);
			pubPoseTag1InvertedRotationMsg.header.frame_id = "world";
			pubPoseTag1InvertedRotationMsg.pose = tagPosesBodyMsg.poses[i];
			pubPoseTag1InvertedRotationMsg.pose.orientation.x = qBT.inverted().x();
			pubPoseTag1InvertedRotationMsg.pose.orientation.y = qBT.inverted().y();
			pubPoseTag1InvertedRotationMsg.pose.orientation.z = qBT.inverted().z();
			pubPoseTag1InvertedRotationMsg.pose.orientation.w = qBT.inverted().w();
			pubPoseTag1InvertedRotation_.publish(pubPoseTag1InvertedRotationMsg);

			geometry_msgs::PoseStamped pubPoseTag1InvertedRotationTranslationMsg;
			pubPoseTag1InvertedRotationTranslationMsg.header.stamp = ros::Time(stateSafe_.t_);
			pubPoseTag1InvertedRotationTranslationMsg.header.frame_id = "world";
			pubPoseTag1InvertedRotationTranslationMsg.pose = pubPoseTag1InvertedRotationMsg.pose;
			pubPoseTag1InvertedRotationTranslationMsg.pose.position.x = -pubPoseTag1InvertedRotationTranslationMsg.pose.position.x;
			pubPoseTag1InvertedRotationTranslationMsg.pose.position.y = -pubPoseTag1InvertedRotationTranslationMsg.pose.position.y;
			pubPoseTag1InvertedRotationTranslationMsg.pose.position.z = -pubPoseTag1InvertedRotationTranslationMsg.pose.position.z;
			pubPoseTag1InvertedRotationTranslation_.publish(pubPoseTag1InvertedRotationTranslationMsg);


			// Get pose/twist and publish
		  Eigen::Vector3d IrIB;
		  rot::RotationQuaternionPD qIB;
		  Eigen::Vector3d BvB;
		  Eigen::Vector3d BwB;
		  getOutput(stateSafe_,IrIB,qIB,BvB,BwB);
		  Eigen::Matrix<double,12,12> Cov;
		  Cov = getOutputCovariance(stateSafe_,stateSafeP_);

		  geometry_msgs::TwistWithCovarianceStamped msgTwist;
		  msgTwist.twist.twist.linear.x = BvB(0);
		  msgTwist.twist.twist.linear.y = BvB(1);
		  msgTwist.twist.twist.linear.z = BvB(2);
		  msgTwist.twist.twist.angular.x = BwB(0);
		  msgTwist.twist.twist.angular.y = BwB(1);
		  msgTwist.twist.twist.angular.z = BwB(2);
		  unsigned int indexArrayTwist[6] = {6,7,8,9,10,11};
		  for(unsigned int i=0;i<6;i++){
			for(unsigned int j=0;j<6;j++){
			  msgTwist.twist.covariance[6*i+j] = Cov(indexArrayTwist[i],indexArrayTwist[j]);
			}
		  }
		  msgTwist.header.stamp = ros::Time(stateSafe_.t_);
		  pubTwistSafe_.publish(msgTwist);
	    }
	}

	pubTagPoses_.publish(tagPosesMsg);
	pubTagPosesBody_.publish(tagPosesBodyMsg);

	geometry_msgs::PoseArray poseArrayMsg;
	poseArrayMsg.poses = tagPosesMsg.poses;
	poseArrayMsg.header = tagPosesMsg.header;
	pubTagVis_.publish(poseArrayMsg);
}

