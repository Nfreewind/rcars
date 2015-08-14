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

std::shared_ptr<tf::TransformBroadcaster> transformBroadcaster;

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

	transformBroadcaster->sendTransform(T_IM);
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

	transformBroadcaster->sendTransform(T_MV);
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

		transformBroadcaster->sendTransform(T_VT);
	}
}

void publish_T_WI(const std_msgs::Header& header)
{
	// Publish the corresponding tf
	tf::StampedTransform T_WI;
	T_WI.frame_id_ = "world";
	T_WI.child_frame_id_ = "rcars_inertial";
	T_WI.stamp_ = header.stamp;

	tf::Transform tf;
	tf.setOrigin(tf::Vector3(0, 0, 0));
	tf.setRotation(tf::Quaternion(0, 0, 0, 1));
	T_WI.setData(tf);

	transformBroadcaster->sendTransform(T_WI);
}

void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose, const geometry_msgs::PoseWithCovarianceStampedConstPtr& extrinsics, const rcars_detector::TagArrayConstPtr& detectedTags)
{
	publish_T_IM(pose);
	publish_T_MV(extrinsics);
	publish_T_VT(detectedTags);

	publish_T_WI(pose->header);
}

int main(int argc, char **argv)
{
	ROS_INFO("Launching RCARS tf_publisher.");

	ros::init(argc, argv, "tf_publisher");
	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate("~");

	transformBroadcaster = std::make_shared<tf::TransformBroadcaster>();

	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> poseSub(nh, "estimator/filterPoseSafe", 2);
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> extrinsicsSub(nh, "estimator/extrinsics", 2);
	message_filters::Subscriber<rcars_detector::TagArray> tagSub(nh, "estimator/tagsCameraFrame", 2);

	typedef message_filters::sync_policies::ExactTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, rcars_detector::TagArray> rcarsStatePublisher;

	message_filters::Synchronizer<rcarsStatePublisher> rcarsState(rcarsStatePublisher(10), poseSub, extrinsicsSub, tagSub);
	rcarsState.registerCallback(boost::bind(&callback, _1, _2, _3));

	ros::spin();
}
