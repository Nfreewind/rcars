#include "ros/ros.h"
#include "ros/console.h"

#include <sensor_msgs/Imu.h>
#include <rcars_detector/TagArray.h>
#include "FilterRCARS.hpp"
#include "FilterInterface_RCARS.hpp"


int main(int argc, char *argv[]){
  // Ros initialization and ros node handle
  ros::init(argc, argv, "estimator");
  ros::NodeHandle n("~");
  ROS_INFO("Launching RCARS estimator. Will be waiting for camera_info afterwards.");

  // Instance of filterInterface
  FilterInterface_RCARS filterInterface(n);

  // Spin
  ros::spin();

  // Save workspace
  bool saveWorkspace = false;
  n.param<bool>("autosaveWorkspace", saveWorkspace, saveWorkspace);

  if (saveWorkspace)
  {
	  ROS_INFO("Saving Workspace.");
	  filterInterface.saveWorkspace();
  }

  return 0;
}
