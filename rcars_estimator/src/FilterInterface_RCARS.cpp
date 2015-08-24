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

#include "FilterInterface_RCARS.hpp"


FilterInterface_RCARS::FilterInterface_RCARS(ros::NodeHandle& nh, ros::NodeHandle& nhRCARS) :
	nh_(nh)
{
  ros::NodeHandle nhNonPrivate;

  // Initialize remaining filter variables
  properVisionDataAvailable_ = false;
  isInitialized_ = false;
  camInfoAvailable_ = false;
  referenceTagId_ = -1;

  nh_.param<int>("calibrationViewCountThreshold", calibrationViewCountThreshold_, 10);
  nh_.param<bool>("overwriteWorkspace", overwriteWorkspace_, false);
  nh_.param<bool>("initializeWithStaticTagOnly", initializeWithStaticTagOnly_, false);

  if(!nhRCARS.getParam("tagSize", std::get<0>(mUpdates_).tagSize_))
  {
	  ROS_FATAL("tagSize parameter not set. Cannot estimate without tagSize. Please add it to the ROS parameter server or configFile");
	  exit(-1);
  }

  std::string filterParameterFile;
  if(nh_.getParam("filterParameterFile", filterParameterFile))
  {
	  readFromInfo(filterParameterFile);
  } else
  {
	  ROS_FATAL("parameter filterParameterFile is unset. Cannot load filter.");
	  exit(-1);
  }
  loadWorkspace();

  // Outlier detection currently disabled
  // enableOutlierDetection();
//  disableOutlierDetection(); // TODO

  // Reset the filter
  reset();

  // Setup subscribers and publishers
  subImu_ = nh_.subscribe("/imu0", 1000, &FilterInterface_RCARS::imuCallback,this);
  subTags_ = nhNonPrivate.subscribe("detector/tags", 10, &FilterInterface_RCARS::visionCallback,this);
  subCameraInfo_ = nh_.subscribe("/cam0/camera_info", 1, &FilterInterface_RCARS::cameraInfoCallback,this);
  pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>("filterPose", 20);
  pubTagArrayCameraFrame_ = nh_.advertise<rcars_detector::TagArray>("tagsCameraFrame", 20);
  pubTagArrayInertialFrame_ = nh_.advertise<rcars_detector::TagArray>("tagsInertialFrame",20);
  pubPoseSafe_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("filterPoseSafe", 20);
  pubTwistSafe_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("filterTwistSafe", 20);
  pubExtrinsics_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("filterExtrinsics", 20);

  resetService_ = nh_.advertiseService("reset", &FilterInterface_RCARS::resetServiceCallback, this);
  saveWorkspaceService_ = nh_.advertiseService("saveWorkspace", &FilterInterface_RCARS::saveWorkspaceCallback, this);
  filterStatusService_ = nh_.advertiseService("getFilterStatus", &FilterInterface_RCARS::getFilterStatusCallback, this);
}


void FilterInterface_RCARS::loadWorkspace()
{
	std::vector<int> calibratedTags;
	if (!nh_.getParam("workspace/calibratedTags", calibratedTags))
	{
		ROS_FATAL("No calibrated tags found. Creating empty workspace.");
		return;
	}

	Eigen::Vector3d WrWT;
	Eigen::Quaterniond qTW;

	if (nh_.getParam("workspace/referenceTagId", referenceTagId_))
	{
		if (std::find(calibratedTags.begin(), calibratedTags.end(), referenceTagId_) == calibratedTags.end() )
		{
			ROS_FATAL("WARNING: Reference tag with id %d is not listed as a calibrated tag.", referenceTagId_);
		}

		if(
			nh_.getParam("workspace/T_workspace_refTag/position/x", WrWT(0)) &&
			nh_.getParam("workspace/T_workspace_refTag/position/y", WrWT(1)) &&
			nh_.getParam("workspace/T_workspace_refTag/position/z", WrWT(2)) &&
			nh_.getParam("workspace/T_workspace_refTag/orientation/w", qTW.w()) &&
			nh_.getParam("workspace/T_workspace_refTag/orientation/x", qTW.x()) &&
			nh_.getParam("workspace/T_workspace_refTag/orientation/x", qTW.y()) &&
			nh_.getParam("workspace/T_workspace_refTag/orientation/y", qTW.z())
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
		std::string parameterBaseName = "workspace/tags/tag" + std::to_string(tagId);
		std::string tagType;
		V3D IrIT;
		QPD qTI;
		if (
			nh_.getParam(parameterBaseName+"/type", tagType) &&
			nh_.getParam(parameterBaseName+"/pose/position/x", IrIT(0)) &&
			nh_.getParam(parameterBaseName+"/pose/position/y", IrIT(1)) &&
			nh_.getParam(parameterBaseName+"/pose/position/z", IrIT(2))
		)
		{
			if (tagType == "static")
			{
				if (nh_.getParam(parameterBaseName+"/pose/orientation/w", qTI.toImplementation().w()) &&
					nh_.getParam(parameterBaseName+"/pose/orientation/x", qTI.toImplementation().x()) &&
					nh_.getParam(parameterBaseName+"/pose/orientation/y", qTI.toImplementation().y()) &&
					nh_.getParam(parameterBaseName+"/pose/orientation/z", qTI.toImplementation().z())
				)
				{
				  addStaticTag(tagId,rcars::STATIC_TAG,IrIT,qTI);
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

void FilterInterface_RCARS::addStaticTag(int tagId, rcars::TagType tagType, const V3D& IrIT, const QPD& qTI){
  tagType_[tagId] = tagType;
  IrIT_[tagId] = IrIT;
  qTI_[tagId] = qTI;
}

void FilterInterface_RCARS::saveWorkspace()
{
  std::string filename;
  if (!nh_.getParam("workspaceFilename", filename))
  {
	filename = ros::package::getPath("rcars_estimator") + "/config/workspaces/default.yaml";
    ROS_WARN("Parameter workspace filename not set. Will save workspace to default location %s.", filename.c_str());
  }

	ROS_INFO("Saving workspace to %s.", filename.c_str());

	// if we do not only update, first delete all tags
	if (overwriteWorkspace_)
	{
		ROS_INFO("Existing workspace will be overwritten.");
		nh_.deleteParam("tags");
	}

	std::vector<int> calibratedTags;
	std::vector<int> calibratedTagIndeces;

	for (size_t i=0; i< mtState::nDynamicTags_; i++)
	{
		int tagId = safe_.state_.template get<mtState::_aux>().dynamicIds_[i];
		if (tagId != -1)
		{
			if (tagViewCount_[tagId] > calibrationViewCountThreshold_)
			{
				calibratedTags.push_back(tagId);
				calibratedTagIndeces.push_back(i);

				ROS_INFO("Adding Tag with id %d and %u views and %u overlapping views.", tagId, tagViewCount_[tagId], tagViewOverlapCount_[tagId]);
			} else
			{
				ROS_INFO("Skipping Tag with id %d and %u views and %u overlapping views.", tagId, tagViewCount_[tagId], tagViewOverlapCount_[tagId]);
			}
		}
	}

	for (size_t i=0; i<calibratedTags.size(); i++)
	{
		int tagId = calibratedTags[i];
		int tagIndex = calibratedTagIndeces[i];

		std::string parameterBaseName = "workspace/tags/tag" + std::to_string(tagId);

		nh_.deleteParam(parameterBaseName);

		nh_.setParam(parameterBaseName+"/type", "static");

		V3D IrIT = get_IrIT_dyn_safe(tagIndex);
		QPD qTI = get_qTI_dyn_safe(tagIndex);

		nh_.setParam(parameterBaseName+"/pose/position/x", IrIT(0));
		nh_.setParam(parameterBaseName+"/pose/position/y", IrIT(1));
		nh_.setParam(parameterBaseName+"/pose/position/z", IrIT(2));
		nh_.setParam(parameterBaseName+"/pose/orientation/w", qTI.w());
		nh_.setParam(parameterBaseName+"/pose/orientation/x", qTI.x());
		nh_.setParam(parameterBaseName+"/pose/orientation/y", qTI.y());
		nh_.setParam(parameterBaseName+"/pose/orientation/z", qTI.z());
	}

	ROS_INFO("%lu newly calibrated tags.", calibratedTags.size());

	// take over old tags
	if (!overwriteWorkspace_)
	{
		std::vector<int> calibratedTagsPreviously;
		nh_.getParam("workspace/calibratedTags", calibratedTagsPreviously);
		for (size_t i=0; i<calibratedTagsPreviously.size(); i++)
		{
			if (std::find(calibratedTags.begin(), calibratedTags.end(), calibratedTagsPreviously[i]) == calibratedTags.end() )
			{
				calibratedTags.push_back(calibratedTagsPreviously[i]);
			}
		}
	}

	nh_.setParam("workspace/calibratedTags", calibratedTags);
	ROS_INFO("%lu total calibrated tags.", calibratedTags.size());

	std::string systemCall = "rosparam dump " + filename + " " + ros::this_node::getNamespace() + "/estimator/workspace";
	if (system(systemCall.c_str()) == -1)
	{
		ROS_FATAL("Could not save workspace to %s", filename.c_str());
	} else{
		ROS_INFO("Saved workspace to %s", filename.c_str());
	}
}

bool FilterInterface_RCARS::resetServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	reset();
	isInitialized_ = false;
	properVisionDataAvailable_ = false;
	return true;
}

bool FilterInterface_RCARS::saveWorkspaceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	saveWorkspace();
	return true;
}

bool FilterInterface_RCARS::getFilterStatusCallback(rcars_estimator::FilterStatus::Request& request, rcars_estimator::FilterStatus::Response& response)
{
	response.currentFilterTime = ros::Time(safe_.t_);
	response.timeSinceLastUpdate = ros::Time(safe_.state_.template get<mtState::_aux>().timeSinceLastValidUpdate_);
	return true;
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

//  if(verbose_) std::cout safe_<< std::setprecision(15) << "== New IMU meas, timestamp: " << imu_msg->header.stamp.toSec() << std::endl;

  // Check if initialization can be performed (requires the availability of tag measurements)
  if(!isInitialized_ && properVisionDataAvailable_) {
      initializeFilterWithIMUMeas(predictionMeas,imu_msg->header.stamp.toSec());
  } else if(isInitialized_){ // If the filter is initialized add the IMU measurement to the filter and update
    addPredictionMeas(predictionMeas,imu_msg->header.stamp.toSec());
    updateAndPublish();
  }
}

void FilterInterface_RCARS::visionCallback(const rcars_detector::TagArray::ConstPtr& vision_msg) {

  // Return if the camera info is not yet available
  if(!camInfoAvailable_) { return; }

  // Do not add measurements if not initialized
  if (!isInitialized_) {
    if(initializeWithStaticTagOnly_){
      for (size_t i=0; i<vision_msg->tags.size(); i++){
        int tagId = vision_msg->tags[i].id;
        auto it = tagType_.find(tagId);
        if (it != tagType_.end() && it->second == rcars::STATIC_TAG){
          properVisionDataAvailable_ = true;
          break;
        }
      }
    } else if(vision_msg->tags.size() > 0){
      properVisionDataAvailable_ = true;
    }
    return;
  }

  // Create and fill the update measurement
  mtUpdateMeas updateMeas;
  updateMeas.template get<mtUpdateMeas::_aux>().resize(vision_msg->tags.size());

  // Read out the tags from the current TagArray measurement
  if(verbose_) std::cout << "== New vision meas, timestamp: " << vision_msg->header.stamp.toSec() << std::endl;
  for (size_t i=0; i<vision_msg->tags.size(); i++){

    int tagId = vision_msg->tags[i].id;
    // Copy the tag index
    updateMeas.template get<mtUpdateMeas::_aux>().tagIds_[i] = tagId;

    tagViewCount_[vision_msg->tags[i].id]++;
    if (vision_msg->tags.size() > 1) { tagViewOverlapCount_[tagId]++; }

    // by default tags are dynamic
    updateMeas.template get<mtUpdateMeas::_aux>().tagTypes_[i] = rcars::DYNAMIC_TAG;

    // check if tag is of special type
    auto it = tagType_.find(tagId);
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
    if (updateMeas.template get<mtUpdateMeas::_aux>().tagTypes_[i] == rcars::STATIC_TAG)
    {
    	updateMeas.template get<mtUpdateMeas::_aux>().IrIT_[i] = IrIT_[tagId];
    	updateMeas.template get<mtUpdateMeas::_aux>().qTI_[i] = qTI_[tagId];
    }
  }

  // Add the update measurement and update
  if(verbose_) updateMeas.print();
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
  // Only update if the filter is initialized
  if (isInitialized_) {
    // Store the current time of the filter
    double t = safe_.t_;
    // Do a safe update, i.e., to the last image msg which has available imu data
    double lastImuTime;
    if(predictionTimeline_.getLastTime(lastImuTime)){
      auto rit = std::get<0>(updateTimelineTuple_).measMap_.rbegin();
      while(rit != std::get<0>(updateTimelineTuple_).measMap_.rend() && rit->first > lastImuTime) ++rit;
      if(rit != std::get<0>(updateTimelineTuple_).measMap_.rend())
      {
    	  const double updateTime = rit->first;
    	  updateSafe(&updateTime);
      }
    }

    // Check if something has changed, if yes publish filter state
    if(safe_.t_>t){

      // Verbose
      if(verbose_) std::cout << "Calibration:" << std::endl;
      if(verbose_) std::cout << "  MrMV: " << safe_.state_.template get<mtState::_vep>().transpose() << std::endl;
      if(verbose_) std::cout << "  qVM: " << safe_.state_.template get<mtState::_vea>() << std::endl;
      if(verbose_) std::cout << "  Time since last valid measurement: " << safe_.state_.template get<mtState::_aux>().timeSinceLastValidUpdate_ << std::endl;

      // Publish further tag poses
      publishTagPoses();

      // Get pose/twist and publish
      Eigen::Vector3d IrIM;
      rot::RotationQuaternionPD qIM;
      Eigen::Vector3d MvM;
      Eigen::Vector3d MwM;
      getOutput(safe_,IrIM,qIM,MvM,MwM);

      rot::RotationQuaternionPD qMI(qIM.inverted());

      Eigen::Matrix<double,12,12> Cov;
      Cov = getOutputCovariance(safe_);

      // Publish pose
      geometry_msgs::PoseWithCovarianceStamped msg;
      msg.pose.pose.position.x = IrIM(0);
      msg.pose.pose.position.y = IrIM(1);
      msg.pose.pose.position.z = IrIM(2);
      msg.pose.pose.orientation.w = qMI.w();
      msg.pose.pose.orientation.x = qMI.x();
      msg.pose.pose.orientation.y = qMI.y();
      msg.pose.pose.orientation.z = qMI.z();
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

      // Publish extrinsics
      geometry_msgs::PoseWithCovarianceStamped msgExtrinsics;
      const V3D& MrMV = safe_.state_.template get<mtState::_vep>();
      const QPD& qVM = safe_.state_.template get<mtState::_vea>();
      msgExtrinsics.pose.pose.position.x = MrMV(0);
      msgExtrinsics.pose.pose.position.y = MrMV(1);
      msgExtrinsics.pose.pose.position.z = MrMV(2);
      msgExtrinsics.pose.pose.orientation.w = qVM.w();
      msgExtrinsics.pose.pose.orientation.x = qVM.x();
      msgExtrinsics.pose.pose.orientation.y = qVM.y();
      msgExtrinsics.pose.pose.orientation.z = qVM.z();
      const int vepId = mtState::template getId<mtState::_vep>();
      const int veaId = mtState::template getId<mtState::_vea>();
      unsigned int indexArrayExt[6] = {vepId, vepId+1, vepId+2, veaId, veaId+1, veaId+2};
      for(unsigned int i=0;i<6;i++){
        for(unsigned int j=0;j<6;j++){
          msgExtrinsics.pose.covariance[6*i+j] = safe_.cov_(indexArrayExt[i],indexArrayExt[j]);
        }
      }
      msgExtrinsics.header.stamp = ros::Time(safe_.t_);
      pubExtrinsics_.publish(msgExtrinsics);
    }
  }
}

void FilterInterface_RCARS::publishTagPoses(void)
{
	// TODO: add static tags here
	rcars_detector::TagArray tagArrayCameraFrameMsg;
	tagArrayCameraFrameMsg.header.stamp = ros::Time(safe_.t_);
	tagArrayCameraFrameMsg.header.frame_id = "world";

	rcars_detector::TagArray tagArrayInertialFrameMsg;
	tagArrayInertialFrameMsg.header.stamp = ros::Time(safe_.t_);
	tagArrayInertialFrameMsg.header.frame_id = "world";

	size_t index = 0;
	for (size_t i=0; i< mtState::nDynamicTags_; i++)
	{
		if (safe_.state_.template get<mtState::_aux>().dynamicIds_[i] == -1)
		{
			continue;
		}

		tagArrayCameraFrameMsg.tags.resize(tagArrayCameraFrameMsg.tags.size()+1);
		tagArrayInertialFrameMsg.tags.resize(tagArrayInertialFrameMsg.tags.size()+1);

		tagArrayCameraFrameMsg.tags[index].id = safe_.state_.template get<mtState::_aux>().dynamicIds_[i];
		tagArrayInertialFrameMsg.tags[index].id = safe_.state_.template get<mtState::_aux>().dynamicIds_[i];

	    tagArrayCameraFrameMsg.tags[index].pose.position.x = get_VrVT_dyn_safe(i)(0);
	    tagArrayCameraFrameMsg.tags[index].pose.position.y = get_VrVT_dyn_safe(i)(1);
	    tagArrayCameraFrameMsg.tags[index].pose.position.z = get_VrVT_dyn_safe(i)(2);
	    tagArrayCameraFrameMsg.tags[index].pose.orientation.x = get_qTV_dyn_safe(i).x();
	    tagArrayCameraFrameMsg.tags[index].pose.orientation.y = get_qTV_dyn_safe(i).y();
	    tagArrayCameraFrameMsg.tags[index].pose.orientation.z = get_qTV_dyn_safe(i).z();
	    tagArrayCameraFrameMsg.tags[index].pose.orientation.w = get_qTV_dyn_safe(i).w();

	    tagArrayInertialFrameMsg.tags[index].pose.position.x = get_IrIT_dyn_safe(i)(0);
	    tagArrayInertialFrameMsg.tags[index].pose.position.y = get_IrIT_dyn_safe(i)(1);
	    tagArrayInertialFrameMsg.tags[index].pose.position.z = get_IrIT_dyn_safe(i)(2);
	    tagArrayInertialFrameMsg.tags[index].pose.orientation.x = get_qTI_dyn_safe(i).x();
	    tagArrayInertialFrameMsg.tags[index].pose.orientation.y = get_qTI_dyn_safe(i).y();
	    tagArrayInertialFrameMsg.tags[index].pose.orientation.z = get_qTI_dyn_safe(i).z();
	    tagArrayInertialFrameMsg.tags[index].pose.orientation.w = get_qTI_dyn_safe(i).w();

	    for (size_t j=0; j<4; j++)
	    {
	    	tagArrayCameraFrameMsg.tags[index].corners[j].x = safe_.state_.template get<mtState::_aux>().corners_[i](j*2+0);
	    	tagArrayCameraFrameMsg.tags[index].corners[j].y = safe_.state_.template get<mtState::_aux>().corners_[i](j*2+1);

	    	tagArrayInertialFrameMsg.tags[index].corners[j].x = safe_.state_.template get<mtState::_aux>().corners_[i](j*2+0);
	    	tagArrayInertialFrameMsg.tags[index].corners[j].y = safe_.state_.template get<mtState::_aux>().corners_[i](j*2+1);
	    }

	    index++;
	}

	pubTagArrayCameraFrame_.publish(tagArrayCameraFrameMsg);
	pubTagArrayInertialFrame_.publish(tagArrayInertialFrameMsg);
}

