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

  // Initialize remaining filter variables
  visionDataAvailable_ = false;
  isInitialized_ = false;
  camInfoAvailable_ = false;
  referenceTagId_ = -1;

  std::string filterParameterFile;
  if(nh.getParam("filterParameterFile", filterParameterFile))
  {
	  readFromInfo(filterParameterFile);
  } else
  {
	  ROS_FATAL("parameter filterParameterFile is unset. Cannot load filter.");
	  exit(-1);
  }
  loadWorkspace(nh);

  // Outlier detection currently disabled
  // enableOutlierDetection();
//  disableOutlierDetection(); // TODO

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
}


void FilterInterface_RCARS::loadWorkspace(ros::NodeHandle& nh)
{
	std::vector<int> calibratedTags;
	if (!nh.getParam("workspace/calibratedTags", calibratedTags))
	{
		ROS_FATAL("No calibrated tags found. Creating empty workspace.");
		return;
	}

	Eigen::Vector3d WrWT;
	Eigen::Quaterniond qTW;

	if (nh.getParam("workspace/referenceTagId", referenceTagId_))
	{
		if (std::find(calibratedTags.begin(), calibratedTags.end(), referenceTagId_) == vector.end() )
		{
			ROS_FATAL("WARNING: Reference tag with id %d is not listed as a calibrated tag.", referenceTagId_);
		}

		if(
			nh.getParam("workspace/T_workspace_refTag/position/x", WrWT(0)) &&
			nh.getParam("workspace/T_workspace_refTag/position/y", WrWT(1)) &&
			nh.getParam("workspace/T_workspace_refTag/position/z", WrWT(2)) &&
			nh.getParam("workspace/T_workspace_refTag/orientation/w", qTW.w()) &&
			nh.getParam("workspace/T_workspace_refTag/orientation/x", qTW.x()) &&
			nh.getParam("workspace/T_workspace_refTag/orientation/x", qTW.y()) &&
			nh.getParam("workspace/T_workspace_refTag/orientation/y", qTW.z())
		)
		{
			ROS_INFO("Found reference Tag and workspace to reference tag transformation.");
			WrWT_ = WrWT;
			qTW_ = rot::RotationQuaternionPD(qTW);
		} else
		{
			ROS_FATAL("Found reference Tag ID but no reference tag transformation. Will assume identity");
			WrWT_.setZero();
			qTW_.setIdentity();
		}
	} else
	{
		ROS_INFO("No workspace reference tag ID found. Cannot give workspace information.");
	}

	for (size_t i=0; i<calibratedTags.size(); i++)
	{
		int tagId = calibratedTags[i];
		std::string parameterBaseName = "tags/tag" + std::to_string(tagId);
		std::string tagType;
		if (
			nh.getParam(parameterBaseName+"/type", tagType) &&
			nh.getParam(parameterBaseName+"/position/x", IrIT_[tagId](0)) &&
			nh.getParam(parameterBaseName+"/position/y", IrIT_[tagId](1)) &&
			nh.getParam(parameterBaseName+"/position/z", IrIT_[tagId](2))
		)
		{
			if (tagType == "static")
			{
				tagType_[tagId] = rcars::STATIC_TAG;
				Eigen::Quaterniond rot;
				if (nh.getParam(parameterBaseName+"/orientation/w", rot.w()) &&
					nh.getParam(parameterBaseName+"/orientation/x", rot.x()) &&
					nh.getParam(parameterBaseName+"/orientation/y", rot.y()) &&
					nh.getParam(parameterBaseName+"/orientation/z", rot.z())
				)
				{
					qTI_[tagId] = rot::RotationQuaternionPD(rot);
				} else
				{
					ROS_FATAL("Could not get orientation for tag %d", tagId);
					exit(-1);
				}
			}
			else
			{
				ROS_FATAL("Unknown tag type of tag %d", tagId);
				exit(-1);
			}
		} else
		{
			ROS_FATAL("Tag type and/or position is unspecified for tag %d", tagId);
			exit(-1);
		}
	}
}


void FilterInterface_RCARS::safeWorkspace(ros::NodeHandle& nh)
{
	ROS_INFO("Saving workspace to %s.", filename);
	std::vector<int> calibratedTags;

	for (size_t i=0; i< safe_.nDynamicTags_; i++)
	{
		int tagId = safe_.template get<mtState::_aux>().dynamicIds_[i];
		if (tagId != -1)
		{
			if (tagViewCount_[tagId] > calibrationThreshold_)
			{
				calibratedTags.push_back(i);
			}
		}
	}

	ROS_INFO("%d dynamic calibrated tags.", calibratedTags.size());

	nh.deleteParam("workspace/calibratedTags");
	nh.setParam("workspace/calibratedTags", calibratedTags);
}

void FilterInterface_RCARS::initializeFilterWithIMUMeas(const mtPredictionMeas& meas, const double& t) {
  // Reset the filter using the provided accelerometer measurement
  resetWithAccelerometer(meas.template get<mtPredictionMeas::_acc>(), t);
  clean(t);
  isInitialized_ = true;
}

void FilterInterface_RCARS::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  // Create and fill prediction measurement using the IMU data
  mtPredictionMeas predictionMeas;
  predictionMeas.template get<mtPredictionMeas::_acc>() = Eigen::Vector3d(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z);
  predictionMeas.template get<mtPredictionMeas::_gyr>() = Eigen::Vector3d(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);

  // Check if initialization can be performed (requires the availability of tag measurements)
  if(!isInitialized_ && visionDataAvailable_) {
      initializeFilterWithIMUMeas(predictionMeas,imu_msg->header.stamp.toSec());
  } else if(isInitialized_){ // If the filter is initialized add the IMU measurement to the filter and update
    addPredictionMeas(predictionMeas,imu_msg->header.stamp.toSec());
    updateAndPublish();
  }
}

void FilterInterface_RCARS::visionCallback(const rcars_detector::TagArray::ConstPtr& vision_msg) {

  visionDataAvailable_ = true;

  // Return if the camera info is not yet available
  if(!camInfoAvailable_) { return; }

  // Do not add measurements if not initialized
  if (!isInitialized_) { return; }

  // Create and fill the update measurement
  mtUpdateMeas updateMeas(vision_msg->tags.size());

  // Read out the tags from the current TagArray measurement
  for (size_t i=0; i<vision_msg->tags.size(); i++){
    // Copy the tag index
    updateMeas.template get<mtUpdateMeas::_aux>().tagIds_[i] = vision_msg->tags[i].id;

    tagViewCount_[vision_msg->tags[i].id]++;
    if (vision_msg->tags.size() > 1) { tagViewOverlapCount_[vision_msg->tags[i].id]++; }

    // by default tags are dynamic
    updateMeas.template get<mtUpdateMeas::_aux>().tagTypes_[i] = rcars::DYNAMIC_TAG;

    // check if tag is of special type
    auto it = tagType_.find(updateMeas.template get<mtUpdateMeas::_aux>().tagIds_[i]);
    if (it != tagType_.end())
    {
    	updateMeas.template get<mtUpdateMeas::_aux>().tagTypes_[i] = it->second;
    }

    // Copy the corner measurements
    for (size_t j=0; j<4; j++){
      updateMeas.template get<mtUpdateMeas::_aux>().VrVC_[i](2*j) = vision_msg->tags[i].corners[j].x;
      updateMeas.template get<mtUpdateMeas::_aux>().VrVC_[i](2*j+1) = vision_msg->tags[i].corners[j].y;
    }

    // Copy the estimated pose from the detector
    const geometry_msgs::Pose& pose = vision_msg->tags[i].pose;
    updateMeas.template get<mtUpdateMeas::_aux>().tagPos_[i] = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    updateMeas.template get<mtUpdateMeas::_aux>().tagAtt_[i] = rot::RotationQuaternionPD(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    // if we have a static tag, we copy the position and orientation data
    if (updateMeas.template get<mtUpdateMeas::_aux>().tagType_[i] == rcars::STATIC_TAG)
    {
    	updateMeas.template get<mtUpdateMeas::_aux>().IrIT_[i] = IrIT_[updateMeas.template get<mtUpdateMeas::_aux>().tagId_];
    	updateMeas.template get<mtUpdateMeas::_aux>().qTI_[i] = qTI_[updateMeas.template get<mtUpdateMeas::_aux>().tagId_];
    }
  }

  // Add the update measurement and update
  addUpdateMeas(updateMeas, vision_msg->header.stamp.toSec());
  updateAndPublish();
}

void FilterInterface_RCARS::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr cameraInfo_msg){
  // Readout the camera matrix from the CameraInfo message
	for (size_t i=0; i<std::get<0>(mUpdates_).CameraMatrix_.RowsAtCompileTime; i++){
		for (size_t j=0; j<std::get<0>(mUpdates_).CameraMatrix_.ColsAtCompileTime; j++){
			std::get<0>(mUpdates_).CameraMatrix_(i,j) = cameraInfo_msg->P[i*4 + j];
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
    double t = safe_.t_;
    // Do a safe update, i.e., to the prediction or update measurement which lies more back in time
    updateSafe();
    // Check if something has changed, if yes publish filter state
    if(safe_.t_>t){
      // get the estimated position and attitude
      Eigen::Vector3d pos = get_IrIM_safe();
      rot::RotationQuaternionPD quat = get_qMI_safe();

      // Publish the corresponding tf
      static tf::TransformBroadcaster tb_ekf_safe;
      tf::StampedTransform tf_transform_ekf;
      tf_transform_ekf.frame_id_ = "world";
      tf_transform_ekf.child_frame_id_ = "ekf_safe";
      tf_transform_ekf.stamp_ = ros::Time(safe_.t_);
      tf::Transform tf;
      tf.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
      tf.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
      tf_transform_ekf.setData(tf);
      tb_ekf_safe.sendTransform(tf_transform_ekf);

      // Publish the pose with timestamp
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time(safe_.t_);
      pose.header.frame_id = "world";
      pose.pose.position.x = pos(0);
      pose.pose.position.y = pos(1);
      pose.pose.position.z = pos(2);
      pose.pose.orientation.x = quat.x();
      pose.pose.orientation.y = quat.y();
      pose.pose.orientation.z = quat.z();
      pose.pose.orientation.w = quat.w();
      pubPose_.publish(pose);

      // Publish further tag poses
      publishTagPoses();

      // Get pose/twist and publish
      Eigen::Vector3d IrIM;
      rot::RotationQuaternionPD qIM;
      Eigen::Vector3d MvM;
      Eigen::Vector3d MwM;
      getOutput(safe_,IrIM,qIM,MvM,MwM);
      Eigen::Matrix<double,12,12> Cov;
      Cov = getOutputCovariance(safe_);

      // Publish pose
      geometry_msgs::PoseWithCovarianceStamped msg;
      msg.pose.pose.position.x = IrIM(0);
      msg.pose.pose.position.y = IrIM(1);
      msg.pose.pose.position.z = IrIM(2);
      msg.pose.pose.orientation.w = qIM.w();
      msg.pose.pose.orientation.x = qIM.x();
      msg.pose.pose.orientation.y = qIM.y();
      msg.pose.pose.orientation.z = qIM.z();
      unsigned int indexArray[6] = {0,1,2,3,4,5};
      for(unsigned int i=0;i<6;i++){
        for(unsigned int j=0;j<6;j++){
          msg.pose.covariance[6*i+j] = Cov(indexArray[i],indexArray[j]);
        }
      }
      msg.header.stamp = ros::Time(safe_.t_);
      pubPoseSafe_.publish(msg);


      // Publish twist
	  geometry_msgs::TwistWithCovarianceStamped msgTwist;
	  msgTwist.twist.twist.linear.x = MvM(0);
	  msgTwist.twist.twist.linear.y = MvM(1);
	  msgTwist.twist.twist.linear.z = MvM(2);
	  msgTwist.twist.twist.angular.x = MwM(0);
	  msgTwist.twist.twist.angular.y = MwM(1);
	  msgTwist.twist.twist.angular.z = MwM(2);
	  unsigned int indexArrayTwist[6] = {6,7,8,9,10,11};
	  for(unsigned int i=0;i<6;i++){
		for(unsigned int j=0;j<6;j++){
		  msgTwist.twist.covariance[6*i+j] = Cov(indexArrayTwist[i],indexArrayTwist[j]);
		}
	  }
	  msgTwist.header.stamp = ros::Time(safe_.t_);
	  pubTwistSafe_.publish(msgTwist);

    }
  }
}

void FilterInterface_RCARS::publishTagPoses(void)
{
	// TODO: add static tags here
	rcars_detector::TagPoses tagPosesMsg;
	tagPosesMsg.header.stamp = ros::Time(safe_.t_);
	tagPosesMsg.header.frame_id = "world";
	tagPosesMsg.tagIds.resize(nTags_);
	tagPosesMsg.poses.resize(nTags_);

	rcars_detector::TagPoses tagPosesBodyMsg;
	tagPosesBodyMsg.header.stamp = ros::Time(safe_.t_);
	tagPosesBodyMsg.header.frame_id = "world";
	tagPosesBodyMsg.tagIds.resize(nTags_);
	tagPosesBodyMsg.poses.resize(nTags_);


	Eigen::Vector3d IrIM;
	rot::RotationQuaternionPD qIM;
	Eigen::Vector3d MvM;
	Eigen::Vector3d MwM;
	getOutput(safe_,IrIM,qIM,MvM,MwM);

//	for(unsigned int i=0;i<nTags_;i++){
//		tagPosesMsg.tagIds[i] = safe_.tagId_(i);
//		tagPosesBodyMsg.tagIds[i] = safe_.tagId_(i);
//
//	    tagPosesMsg.poses[i].position.x = safe_.tagPos(i)(0);
//	    tagPosesMsg.poses[i].position.y = safe_.tagPos(i)(1);
//	    tagPosesMsg.poses[i].position.z = safe_.tagPos(i)(2);
//	    tagPosesMsg.poses[i].orientation.x = safe_.tagAtt(i).x();
//	    tagPosesMsg.poses[i].orientation.y = safe_.tagAtt(i).y();
//	    tagPosesMsg.poses[i].orientation.z = safe_.tagAtt(i).z();
//	    tagPosesMsg.poses[i].orientation.w = safe_.tagAtt(i).w();
//
//	    Eigen::Vector3d IrMT = safe_.tagPos(i) - IrIM;
//	    Eigen::Vector3d MrMT = qIM.inverted().rotate(IrMT);
//
//	    tagPosesBodyMsg.poses[i].position.x = MrMT(0);
//	    tagPosesBodyMsg.poses[i].position.y = MrMT(1);
//	    tagPosesBodyMsg.poses[i].position.z = MrMT(2);
//
//	    rot::RotationQuaternionPD qMT = qIM.inverted() * safe_.tagAtt(i).inverted();
//
//	    tagPosesBodyMsg.poses[i].orientation.x = qMT.x();
//	    tagPosesBodyMsg.poses[i].orientation.y = qMT.y();
//	    tagPosesBodyMsg.poses[i].orientation.z = qMT.z();
//	    tagPosesBodyMsg.poses[i].orientation.w = qMT.w();
//	}

	pubTagPoses_.publish(tagPosesMsg);
	pubTagPosesBody_.publish(tagPosesBodyMsg);

	// We publish again as a pose array just to make it easier to visualize in RVIZ
	geometry_msgs::PoseArray poseArrayMsg;
	poseArrayMsg.poses = tagPosesMsg.poses;
	poseArrayMsg.header = tagPosesMsg.header;
	pubTagVis_.publish(poseArrayMsg);
}

