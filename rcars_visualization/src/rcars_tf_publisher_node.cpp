/*
 * tf_publisher_node.cpp
 *
 *  Created on: 14.08.2015
 *      Author: neunertm
 */

#include <memory>

#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <rcars_detector/TagArray.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

std::shared_ptr<tf::TransformBroadcaster> pTransformBroadcaster;
std::shared_ptr<tf::TransformListener> pListener;

bool do_align_frames = false;
std::string frame_rcars;
std::string frame_align;

struct StaticTag
{
	int tagId;
	std::string tagType;
	tf::Vector3 IrIT;
	tf::Quaternion qTI;
};

std::vector<StaticTag> staticTags;

void loadWorkspace(ros::NodeHandle& nh)
{
	std::vector<int> calibratedTags;
	if (!nh.getParam("estimator/workspace/calibratedTags", calibratedTags))
	{
		ROS_INFO("No calibrated tags found.");
		return;
	}

	for (size_t i=0; i<calibratedTags.size(); i++)
	{
		StaticTag tag;

		tag.tagId = calibratedTags[i];

		std::string parameterBaseName = "estimator/workspace/tags/tag" + std::to_string(tag.tagId);

		double posx, posy, posz;

		if (
			nh.getParam(parameterBaseName+"/type", tag.tagType) &&
			nh.getParam(parameterBaseName+"/pose/position/x", posx) &&
			nh.getParam(parameterBaseName+"/pose/position/y", posy) &&
			nh.getParam(parameterBaseName+"/pose/position/z", posz)
		)
		{
			if (tag.tagType == "static")
			{
				double w, x, y, z;
				if (nh.getParam(parameterBaseName+"/pose/orientation/w", w) &&
					nh.getParam(parameterBaseName+"/pose/orientation/x", x) &&
					nh.getParam(parameterBaseName+"/pose/orientation/y", y) &&
					nh.getParam(parameterBaseName+"/pose/orientation/z", z)
				)
				{
				  tag.qTI = tf::Quaternion(x,y,z,w);
				  tag.IrIT = tf::Vector3(posx,posy,posz);
				  staticTags.push_back(tag);
				} else
				{
					ROS_FATAL("Could not get orientation for tag %d", tag.tagId);
				}
			}
			else
			{
				ROS_FATAL("Unknown tag type of tag %d", tag.tagId);
			}
		} else
		{
			ROS_FATAL("Tag type and/or position is unspecified for tag %d", tag.tagId);
		}
	}
}

void publish_T_IM(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
	// Publish the corresponding tf
	tf::StampedTransform T_IM;
	T_IM.frame_id_ = "rcars_inertial";
	T_IM.child_frame_id_ = "rcars_IMU";
	T_IM.stamp_ = pose->header.stamp;

	tf::Transform tf;
	tf.setOrigin(tf::Vector3(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z));
	tf.setRotation(tf::Quaternion(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w));
	T_IM.setData(tf);

	pTransformBroadcaster->sendTransform(T_IM);
}

void publish_T_MV(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
	// Publish the corresponding tf
	tf::StampedTransform T_MV;
	T_MV.frame_id_ = "rcars_IMU";
	T_MV.child_frame_id_ = "rcars_camera";
	T_MV.stamp_ = pose->header.stamp;

	tf::Transform tf;
	tf.setOrigin(tf::Vector3(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z));
	tf.setRotation(tf::Quaternion(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w));
	T_MV.setData(tf);

	pTransformBroadcaster->sendTransform(T_MV);
}

void publish_T_VT(const rcars_detector::TagArrayConstPtr& detectedTags)
{
	for (size_t i=0; i<detectedTags->tags.size(); i++)
	{
		// Publish the corresponding tf
		tf::StampedTransform T_VT;
		T_VT.frame_id_ = "rcars_camera";
		T_VT.child_frame_id_ = "rcars_tag" + std::to_string(detectedTags->tags[i].id) + "_dyn";
		T_VT.stamp_ = detectedTags->header.stamp;

		tf::Transform tf;
		tf.setOrigin(tf::Vector3(detectedTags->tags[i].pose.position.x, detectedTags->tags[i].pose.position.y, detectedTags->tags[i].pose.position.z));
		tf.setRotation(tf::Quaternion(detectedTags->tags[i].pose.orientation.x, detectedTags->tags[i].pose.orientation.y, detectedTags->tags[i].pose.orientation.z, detectedTags->tags[i].pose.orientation.w));
		T_VT.setData(tf);

		pTransformBroadcaster->sendTransform(T_VT);
	}
}

void publish_static_tags(const std_msgs::Header& header)
{
	for (size_t i=0; i<staticTags.size(); i++)
	{
		// Publish the corresponding tf
		tf::StampedTransform T_IT;
		T_IT.frame_id_ = "rcars_inertial";
		T_IT.child_frame_id_ = "rcars_tag" + std::to_string(staticTags[i].tagId) + "_static";
		T_IT.stamp_ = header.stamp;

		tf::Transform tf;
		tf.setOrigin(staticTags[i].IrIT);
		tf.setRotation(staticTags[i].qTI);
		T_IT.setData(tf);

		pTransformBroadcaster->sendTransform(T_IT);
	}
}

void publish_T_WW(const std_msgs::Header& header)
{
	// Publish the corresponding tf
	tf::StampedTransform T_WW;
	T_WW.frame_id_ = "world";
	T_WW.child_frame_id_ = "rcars_workspace";
	T_WW.stamp_ = header.stamp;

	tf::Transform tf;
	tf.setOrigin(tf::Vector3(0, 0, 0));
	tf.setRotation(tf::Quaternion(0, 0, 0, 1));
	T_WW.setData(tf);

	pTransformBroadcaster->sendTransform(T_WW);
}

/**
 *   \brief This function aligns two frames and publishes the transformation "workspace->align1=align2->inertial"
 *   \param header of the published new rcars data, used for timing information only.
 *   \return void
 */
void publish_T_WI(const std_msgs::Header& header) {
	//the transformation from workspace frame to alignment 1
	tf::StampedTransform T_WA1;
	//the transformation from inertial frame to alignment 2
	tf::StampedTransform T_A2I;
	//the transformation we want to calculate: from workspace to inertial frame
	tf::StampedTransform T_WI;
	//get T_WA1 and T_A2I
	try {
		pListener->waitForTransform("/rcars_workspace", "/"+frame_align, header.stamp, ros::Duration(1.0));
		pListener->lookupTransform("/rcars_workspace", "/"+frame_align, header.stamp, T_WA1);
		pListener->waitForTransform("/"+frame_rcars, "/rcars_inertial", header.stamp, ros::Duration(1.0));
		pListener->lookupTransform("/"+frame_rcars, "/rcars_inertial", header.stamp, T_A2I);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("COULDN'T align frames! Nothing will be published! Try again in 3 seconds! Error Message: %s.",ex.what());
		ros::Duration(3.0).sleep();
		return;
	}

	//calculate T_WI = T_WA1*T_A2I
	T_WI.frame_id_ = "rcars_workspace";
	T_WI.child_frame_id_ = "rcars_inertial";
	T_WI.stamp_ = header.stamp;
	T_WI.setData(T_WA1*T_A2I);

	//publish data
	pTransformBroadcaster->sendTransform(T_WI);
}

/**
 *	\brief This function sets the Workspace frame the same as the inertial frame.
 *	This function is executed if frame alignment is not desired. (do_frame_alignment = false).
 *	\param header of the published new rcars data, used for timing information only.
 * 	\return void
 */
void publish_T_WI_AS_EQUAL(const std_msgs::Header& header) {
	// Publish the corresponding tf
	tf::StampedTransform T_WI;
	T_WI.frame_id_ = "rcars_workspace";
	T_WI.child_frame_id_ = "rcars_inertial";
	T_WI.stamp_ = header.stamp;

	tf::Transform tf;
	tf.setOrigin(tf::Vector3(0, 0, 0));
	tf.setRotation(tf::Quaternion(0, 0, 0, 1));
	T_WI.setData(tf);

	pTransformBroadcaster->sendTransform(T_WI);
}

void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose, const geometry_msgs::PoseWithCovarianceStampedConstPtr& extrinsics, const rcars_detector::TagArrayConstPtr& detectedTags)
{
	publish_T_IM(pose);
	publish_T_MV(extrinsics);
	publish_T_VT(detectedTags);
	publish_static_tags(detectedTags->header);

	publish_T_WW(pose->header);
	//check if frame wants to be aligned.
	if(do_align_frames) {
		publish_T_WI(pose->header);
	}
	else {
		publish_T_WI_AS_EQUAL(pose->header);
	}
}

int main(int argc, char **argv)
{
	ROS_INFO("Launching RCARS tf_publisher.");

	ros::init(argc, argv, "tf_publisher");
	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate("~");

	loadWorkspace(nh);

	pTransformBroadcaster = std::make_shared<tf::TransformBroadcaster>();
	pListener = std::make_shared<tf::TransformListener>();

	ros::param::param<bool>("~align_workspace", do_align_frames, false);
	ros::param::param<std::string>("~frame_rcars", frame_rcars, "");
	ros::param::param<std::string>("~frame_align", frame_align, "");

	if(!do_align_frames) {
		ROS_WARN("Align workspace set to false. No frame aligment will be performed.");
	}

	if((frame_rcars.empty() || frame_align.empty()) && do_align_frames) {
		ROS_WARN("trying to align frames but got empty string for one or both frame IDs. Setting alignment to false");
		do_align_frames = false;
	}
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> poseSub(nh, "estimator/filterPoseSafe", 2);
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> extrinsicsSub(nh, "estimator/filterExtrinsics", 2);
	message_filters::Subscriber<rcars_detector::TagArray> tagSub(nh, "estimator/tagsCameraFrame", 2);

	typedef message_filters::sync_policies::ExactTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, rcars_detector::TagArray> rcarsStatePublisher;

	message_filters::Synchronizer<rcarsStatePublisher> rcarsState(rcarsStatePublisher(10), poseSub, extrinsicsSub, tagSub);
	rcarsState.registerCallback(boost::bind(&callback, _1, _2, _3));

	ros::spin();
}
