#include <ros/ros.h>
#include <ros/console.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/poses/eigen/HomogeneousTransformation.hpp>

#include <apriltag/TagDetector.hpp>
#include <apriltag/TagFamily.hpp>
#include <apriltag/TagFamilyFactory.hpp>
#include <apriltag/Tag36h11.hpp>
#include <apriltag/Tag25h9.hpp>
#include <apriltag/Tag25h7.hpp>
#include <apriltag/Tag16h5.hpp>

#include "rcars_detector/Tag.h"
#include "rcars_detector/TagArray.h"

#include <sstream>
#include <math.h>
#include <atomic>

using namespace std;

namespace AprilTags = april::tag;

// namespace aliasing for kindr library
namespace rot = kindr::rotations::eigen_impl;
namespace pose = kindr::poses::eigen_impl;
namespace phys = kindr::phys_quant::eigen_impl;

// helper function to convert a number to a string
template <typename T>
string toString ( T Number )
{
 ostringstream ss;
 ss << Number;
 return ss.str();
}

// publisher for detected tags
ros::Publisher tagPublisher;

// subscriber for camera info
ros::Subscriber cameraInfoSubscriber;

// subscribes to the input image
image_transport::Subscriber imageSubscriber;

// publishes debug images with marked detections
image_transport::Publisher debugPublisher;

// pointer to gray-scale image
cv_bridge::CvImagePtr cv_ptr_gray;

// pointer to instance of the tag detecter
AprilTags::TagDetector* tagDetector;

// ROS parameters
bool publishDebugImage = false;
std::string tagFamily = "";
double tagSize = 0.161;

// Camera parameters
sensor_msgs::CameraInfo camInfo;
bool camInfoAvailable = false;

std::atomic<uint_fast32_t> processedSequence(0);
std::atomic<uint8_t> speedWarningCounter(0);


/** \brief Takes the detector information and converts it into a ros message
 * \param detections Detection information containing information such as tagIDs, corners of detected tags
 * \param tagsMessage ROS message to be filled
 * */
void fillRosMessage(const vector<AprilTags::TagDetection>& detections, rcars_detector::TagArray& tagsMessage)
{
	// create an empty container
	rcars_detector::Tag tag;

	// clear the list of tags in the message
	tagsMessage.tags.clear();

	// cycle through all detected tags
	for(size_t i=0; i<detections.size(); ++i)
	{
		// shortcut for the tag in this iteration
		const AprilTags::TagDetection &dd = detections[i];

		// set the tag id in the message
		tag.id = dd.id;

		// Fill in the corner points
		tag.corners[rcars_detector::Tag::BOTTOM_LEFT].x = dd.p[0][0];
		tag.corners[rcars_detector::Tag::BOTTOM_LEFT].y = dd.p[0][1];
		tag.corners[rcars_detector::Tag::BOTTOM_LEFT].z = 0;
		tag.corners[rcars_detector::Tag::BOTTOM_RIGHT].x = dd.p[1][0];
		tag.corners[rcars_detector::Tag::BOTTOM_RIGHT].y = dd.p[1][1];
		tag.corners[rcars_detector::Tag::BOTTOM_RIGHT].z = 0;
		tag.corners[rcars_detector::Tag::TOP_RIGHT].x = dd.p[2][0];
		tag.corners[rcars_detector::Tag::TOP_RIGHT].y = dd.p[2][1];
		tag.corners[rcars_detector::Tag::TOP_RIGHT].z = 0;
		tag.corners[rcars_detector::Tag::TOP_LEFT].x = dd.p[3][0];
		tag.corners[rcars_detector::Tag::TOP_LEFT].y = dd.p[3][1];
		tag.corners[rcars_detector::Tag::TOP_LEFT].z = 0;


		// if we have the camera calibration, we can also output transformations
		if (camInfoAvailable)
		{
			// get the transformation matrix between camera and tag
			Eigen::Matrix4d T_ct= dd.getRelativeTransform(tagSize, camInfo.P[0], camInfo.P[5], camInfo.P[2], camInfo.P[6]);

			// extract the translation vector and normalize the homogeneous coordinates
			phys::Position3D pos(T_ct.block<3,1>(0,3)/T_ct(3,3));

			// extract the rotatoin matrix
			rot::RotationMatrixPD rotationMatrix(T_ct.block<3,3>(0,0));

			// convert the rotation matrix to a quaternion
			rot::RotationQuaternionPD quat(rotationMatrix);
			quat.invert();

			// combine again to a transformation using kindr
			pose::HomogeneousTransformationPosition3RotationQuaternionD trans(pos, quat);

			// copy the translation
			tag.pose.position.x = trans.getPosition().x();
			tag.pose.position.y = trans.getPosition().y();
			tag.pose.position.z = trans.getPosition().z();

			// copy the orientation
			tag.pose.orientation.x = trans.getRotation().x();
			tag.pose.orientation.y = trans.getRotation().y();
			tag.pose.orientation.z = trans.getRotation().z();
			tag.pose.orientation.w = trans.getRotation().w();
		}

		// add tag to the list of detected tags
		tagsMessage.tags.push_back(tag);
	}
}

/** \brief Visulaizes the detections by marking corners and printing the tag id in the image
 * \param detections Detection information containing information such as tagIDs, corners of detected tags
 * \param image Reference to output image to draw on
 * \param Header information from the tag to copy the time stamp from
 * */
void visualizeDetections(const vector<AprilTags::TagDetection>& detections, cv::Mat& image, const std_msgs::Header& header)
{
	// create a new image pointer
	cv_bridge::CvImagePtr cvImage(new cv_bridge::CvImage);

	// set gray-scale and time
	cvImage->encoding = "rgb8";
	cvImage->header = header;

	// get reference to OpenCV implementation
	cv::Mat& debugImage = cvImage->image;

	// convert color
	cvtColor(image, debugImage, CV_GRAY2RGB);

	// iterate over detections
	for(size_t i=0; i<detections.size(); ++i)
	{
		// shortcut
		const AprilTags::TagDetection &dd = detections[i];

		// calculate distance between corners
		int dx = dd.p[0][0]-dd.p[2][0];
		int dy = dd.p[0][1]-dd.p[2][1];

		// set the disparity according to diagonal
		int disparity = sqrt(dx*dx + dy*dy);

		// set corner circle size, thickness and tag id font size depending on disparity
		int circleSize = disparity/15;
		int circleThickness = sqrt(disparity/30);
		double fontSize = disparity/100.0;

		// threshholding
		if (circleSize < 5) { circleSize = 5; }
		if (circleThickness < 2) { circleThickness = 2; }
		if (fontSize < 0.5) { fontSize = 0.5; }
		if (fontSize > 4.0) { fontSize = 4.0; }

		// Draw circles at corners of detected tags on the video stream
		cv::circle(debugImage, cv::Point(dd.p[0][0], dd.p[0][1]), circleSize, CV_RGB(255,0,0), circleThickness);
		cv::circle(debugImage, cv::Point(dd.p[1][0], dd.p[1][1]), circleSize, CV_RGB(255,0,0), circleThickness);
		cv::circle(debugImage, cv::Point(dd.p[2][0], dd.p[2][1]), circleSize, CV_RGB(255,0,0), circleThickness);
		cv::circle(debugImage, cv::Point(dd.p[3][0], dd.p[3][1]), circleSize, CV_RGB(255,0,0), circleThickness);

		// Draw tag id
		cv::putText(
				debugImage,
				toString(dd.id),
				cv::Point(0.5*(dd.p[0][0]+dd.p[2][0]), 0.5*(dd.p[0][1]+dd.p[2][1])),
				cv::FONT_HERSHEY_SIMPLEX,
				fontSize,
				CV_RGB(0,255,0),
				2
		);
	}

    // Publish Image
	debugPublisher.publish(cvImage->toImageMsg());
}

/** \brief Callback function to handle incoming camera information
 * \param cameraInfo CameraInfo (calibration etc.) received by subscriber
 * */
void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cameraInfo)
{
	// copy the info to global instance
	camInfo = *cameraInfo;

	// check if this is the first time to receive camera data. If so, print a message.
	if (!camInfoAvailable)
	{
		ROS_INFO("Received camera info. Will now publish pose estimates for tags as well.");
		camInfoAvailable = true;
	}
}

/** \brief Callback function to handle incoming IMU data
 * \param imageTransport The incoming image message
 * */
void imageCallback(const sensor_msgs::ImageConstPtr& imageTransport)
{
	ROS_DEBUG("received new image");

	uint32_t lastSequence = processedSequence.exchange(imageTransport->header.seq);

	if (imageTransport->header.seq != lastSequence+1)
	{
		speedWarningCounter++;

		if (speedWarningCounter.load() % 10 == 0)
		{
			ROS_WARN("Detector running too slow, dropping images. Consider enabling multithreading or lowering the frame rate or resolution of camera images.");
		}
	}

	// convert incoming image to OpenCV
	try
	{
		cv_ptr_gray = cv_bridge::toCvCopy(imageTransport, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// get a short reference to the image
	cv::Mat& image = cv_ptr_gray->image;

	// Create vector for detections (used for apriltag library)
	vector<AprilTags::TagDetection> detections;

	ROS_DEBUG("detecting tags");
	// Extract Tag Info from cv::Mat image and save it in detections
	tagDetector->process(image, detections);//cv_mat_image_gray_undistorted
	ROS_DEBUG_STREAM("detected "<<detections.size()<<" tags");

	// define ROS message to send out
	rcars_detector::TagArray tags;

	// fill the ROS message with detection infomormation
	fillRosMessage(detections, tags);

	// if visualization is enabled
	if (publishDebugImage)
	{
		// visualize the detections
		visualizeDetections(detections, image, imageTransport->header);
	}

	// Copy header
	tags.header = imageTransport->header;

	// Publish tag information
	tagPublisher.publish(tags);
}

/** \brief Small helper function to select the tag family
 * \param families Vector containing all tag families (currently only one at a time is supported
 * \param string containing the selected tag family
 * */
void selectTagFamily(std::vector<cv::Ptr<AprilTags::TagFamily> >& families, const std::string& tagFamily)
{
	AprilTags::TagFamilyFactory::create(tagFamily, families);
}


/** \brief Main function
 * */

int main(int argc, char **argv)
{
	ROS_INFO("Launching AprilTag Detector.");

	ros::init(argc, argv, "rcars_tag_detector");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	// multi threading
	bool useMultiThreading = false;
	int nThreads = 0;
	ros::param::param<bool>("~useMultiThreading", useMultiThreading, false);
	ros::param::param<int>("~threadsForMultiThreading", nThreads, 0);

	// subscribe to images
	int imageQueueSize = 2;
	if (nThreads > 1)
		imageQueueSize = nThreads*2;
	imageSubscriber = it.subscribe("/cam0/image_rect", imageQueueSize, imageCallback);

	// subscribe to camera info
	cameraInfoSubscriber = nh.subscribe("/cam0/camera_info", 5, cameraInfoCallback);

	// publisher for detected tags
	tagPublisher = nh.advertise<rcars_detector::TagArray>("/rcars_detected_tags", 1);

	// publisher for image with marked corners (for debugging)
	debugPublisher = it.advertise("/rcars_detector_image", 1);

	// select tag family
	int tagFamilyInt = 0;
	ros::param::param<int>("~tagFamily", tagFamilyInt, 4);
	tagFamily = std::to_string(tagFamilyInt);
	std::vector<cv::Ptr<AprilTags::TagFamily> > tagCodes;
	selectTagFamily(tagCodes, tagFamily);
	ROS_INFO("Using tag family %s.", tagFamily.c_str());

	// setup detector
	tagDetector = new AprilTags::TagDetector(tagCodes);
	ros::param::param<double>("~tagSize", tagSize, tagSize);

	// setup debug
	ros::param::param<bool>("~publishDebugImage", publishDebugImage, false);
	ROS_INFO("Debug print is (enabled/disabled): %i", publishDebugImage);

	if (nThreads < 0)
	{
		std::cout<<"Number of threads to use should not be negative"<<std::endl;
		return 1;
	}

	// Call imageCallback function repeatedly
	if (!useMultiThreading)
	{
	   ros::spin();
	} else
	{
		// asynchronous spinner for multithreading
		ros::AsyncSpinner spinner(nThreads);
		spinner.start();
		ros::waitForShutdown();
	}

	// destruct the tag detector instance
	delete tagDetector;

	// everything hopefully went well
	return 0;
}
