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
#include <message_filters/sync_policies/approximate_time.h>

#include <rcars_detector/TagArray.h>

std::map<int, bool> tagSeen;
std::map<int, double> tagLastSeen;

void callbackTags(const nav_msgs::OdometryConstPtr& pose, const geometry_msgs::PoseWithCovarianceStampedConstPtr& extrinsics, const rcars_detector::TagArrayConstPtr& detectedTags)
{

}

int main(int argc, char **argv)
{
	ROS_INFO("Launching RCARS loop_closure_evaluation.");

	ros::init(argc, argv, "loop_closure_evaluation");
	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate("~");

	ros::param::param<double>("~loop_closure_threshold", loop_closure_threshold, "");

	message_filters::Subscriber<rcars_detector::TagArray> tagsDetectorSub(nh, "detector/tags", 30);
	message_filters::Subscriber<rcars_detector::TagArray> tagsEstimatorSub(nh, "estimator/tagsCameraFrame", 30);

	ros::Subscriber sub = nh.subscribe("detector/tags", 2, detectorCallback);

	typedef message_filters::sync_policies::ApproximateTime<rcars_detector::TagArray, rcars_detector::TagArray> SyncPolicy;

	message_filters::Synchronizer<DetectorSyncPolicy> syncDetector(SyncPolicy(10), tagsDetectorSub, tagsEstimatorSub);
	syncDetector.registerCallback(boost::bind(&callbackTags, _1, _2));

	ros::spin();
}
