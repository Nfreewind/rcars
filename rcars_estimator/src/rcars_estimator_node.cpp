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
	  std::cout << "Saving WORKSPACE!!!" << std::endl;
	  filterInterface.saveWorkspace();
  }

  return 0;
}
