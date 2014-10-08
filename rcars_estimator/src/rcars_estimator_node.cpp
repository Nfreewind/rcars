#include "ros/ros.h"
#include "ros/console.h"

#include <sensor_msgs/Imu.h>
#include <rcars_detector/TagArray.h>
#include "FilterRCARS.hpp"
#include "ParameterLoader.hpp"
#include "FilterInterface_RCARS.hpp"

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg){
}


void tagArrayCallback(const rcars_detector::TagArray::ConstPtr &msg){
}


int main(int argc, char *argv[]){
  // Ros initialization and ros node handle
  ros::init(argc, argv, "rcars_estimator");
  ros::NodeHandle n;
  ROS_INFO("Launching RCARS estimator. Will be waiting for camera_info afterwards.");

  // Instance of filterInterface
  FilterInterface_RCARS filterInterface(n);

  // Spin
  ros::spin();
  return 0;
}
