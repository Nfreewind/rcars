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

#ifndef PARAMETERLOADER_HPP_
#define PARAMETERLOADER_HPP_

#include "FilterRCARS.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>

namespace rot = kindr::rotations::eigen_impl;

static void loadVector(Eigen::Vector3d& vec, const std::string &name){
  if(!ros::param::get(name + "x", vec(0))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "y", vec(1))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "z", vec(2))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
}
static void loadQuaternion(rot::RotationQuaternionPD& q, const std::string &name){
  double w, x, y, z;
  if(!ros::param::get(name + "w", w)) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "x", x)) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "y", y)) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "z", z)) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };

  q.setValues(w,x,y,z);
}
static void loadMatrix(Eigen::Matrix3d& mat, const std::string &name){
  if(!ros::param::get(name + "00", mat(0,0))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "01", mat(0,1))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "02", mat(0,2))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "10", mat(1,0))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "11", mat(1,1))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "12", mat(1,2))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "20", mat(2,0))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "21", mat(2,1))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
  if(!ros::param::get(name + "22", mat(2,2))) { ROS_ERROR("parameter %s does not exist", name.c_str()); ROS_BREAK(); };
}
template<unsigned int nTags>
void LoadParameters(const std::string &filename, FilterRCARS::Filter<nTags>* pFilter){
  typedef FilterRCARS::Filter<nTags> mtFilter;

  loadVector(pFilter->BrBM_,"~IMU/BrBM");
  loadQuaternion(pFilter->qMB_,"~IMU/qMB");
  loadVector(pFilter->BrBV_,"~Tag/BrBV");
  loadQuaternion(pFilter->qVB_,"~Tag/qVB");

  if(!ros::param::get("~Tag/OutlierThreshold", pFilter->mTh_)) { ROS_BREAK(); };
  if(!ros::param::get("~Tag/tagSize", pFilter->tagSize_)) { ROS_BREAK(); };

  loadVector(pFilter->initState_.pos(),"~States/pos/init");
  loadVector(pFilter->initState_.vel(),"~States/vel/init");
  loadQuaternion(pFilter->initState_.att(),"~States/att/init");
  loadVector(pFilter->initState_.acb(),"~States/acb/init");
  loadVector(pFilter->initState_.gyb(),"~States/gyb/init");

  for(unsigned int i=0;i<nTags;i++){
    loadVector(pFilter->initState_.tagPos(i),"~States/IrIT/init");
    loadQuaternion(pFilter->initState_.tagAtt(i),"~States/qTI/init");
  }

  double accBiasInitStd;
  if(!ros::param::get("~IMU/AccBiasInitStd", accBiasInitStd)) { ROS_BREAK(); };

  double gyrBiasInitStd;
  if(!ros::param::get("~IMU/GyrBiasInitStd", gyrBiasInitStd)) { ROS_BREAK(); };

  double  pos_initStdx;
  double  pos_initStdy;
  double  pos_initStdz;
  double  vel_initStdx;
  double  vel_initStdy;
  double  vel_initStdz;
  double  att_initStdx;
  double  att_initStdy;
  double  att_initStdz;
  double  IrIT_initStdx;
  double  IrIT_initStdy;
  double  IrIT_initStdz;
  double  qTI_initStdx;
  double  qTI_initStdy;
  double  qTI_initStdz;

  if(!ros::param::get("~States/pos/initStdx", pos_initStdx)) { ROS_BREAK(); };
  if(!ros::param::get("~States/pos/initStdy", pos_initStdy)) { ROS_BREAK(); };
  if(!ros::param::get("~States/pos/initStdz", pos_initStdz)) { ROS_BREAK(); };
  if(!ros::param::get("~States/vel/initStdx", vel_initStdx)) { ROS_BREAK(); };
  if(!ros::param::get("~States/vel/initStdy", vel_initStdy)) { ROS_BREAK(); };
  if(!ros::param::get("~States/vel/initStdz", vel_initStdz)) { ROS_BREAK(); };
  if(!ros::param::get("~States/att/initStdx", att_initStdx)) { ROS_BREAK(); };
  if(!ros::param::get("~States/att/initStdy", att_initStdy)) { ROS_BREAK(); };
  if(!ros::param::get("~States/att/initStdz", att_initStdz)) { ROS_BREAK(); };
  if(!ros::param::get("~States/IrIT/initStdx", IrIT_initStdx)) { ROS_BREAK(); };
  if(!ros::param::get("~States/IrIT/initStdy", IrIT_initStdy)) { ROS_BREAK(); };
  if(!ros::param::get("~States/IrIT/initStdz", IrIT_initStdz)) { ROS_BREAK(); };
  if(!ros::param::get("~States/qTI/initStdx", qTI_initStdx)) { ROS_BREAK(); };
  if(!ros::param::get("~States/qTI/initStdy", qTI_initStdy)) { ROS_BREAK(); };
  if(!ros::param::get("~States/qTI/initStdz", qTI_initStdz)) { ROS_BREAK(); };


  pFilter->initStateP_.setIdentity();
  pFilter->initStateP_(mtFilter::mtState::posInd()+0,mtFilter::mtState::posInd()+0) = pow(pos_initStdx,2);
  pFilter->initStateP_(mtFilter::mtState::posInd()+1,mtFilter::mtState::posInd()+1) = pow(pos_initStdy,2);
  pFilter->initStateP_(mtFilter::mtState::posInd()+2,mtFilter::mtState::posInd()+2) = pow(pos_initStdz,2);
  pFilter->initStateP_(mtFilter::mtState::velInd()+0,mtFilter::mtState::velInd()+0) = pow(vel_initStdx,2);
  pFilter->initStateP_(mtFilter::mtState::velInd()+1,mtFilter::mtState::velInd()+1) = pow(vel_initStdy,2);
  pFilter->initStateP_(mtFilter::mtState::velInd()+2,mtFilter::mtState::velInd()+2) = pow(vel_initStdz,2);
  pFilter->initStateP_.template block<3,3>(mtFilter::mtState::acbInd(),mtFilter::mtState::acbInd()) = Eigen::Matrix3d::Identity()*pow(accBiasInitStd,2);
  pFilter->initStateP_.template block<3,3>(mtFilter::mtState::gybInd(),mtFilter::mtState::gybInd()) = Eigen::Matrix3d::Identity()*pow(gyrBiasInitStd,2);
  pFilter->initStateP_(mtFilter::mtState::attInd()+0,mtFilter::mtState::attInd()+0) = pow(att_initStdx,2);
  pFilter->initStateP_(mtFilter::mtState::attInd()+1,mtFilter::mtState::attInd()+1) = pow(att_initStdy,2);
  pFilter->initStateP_(mtFilter::mtState::attInd()+2,mtFilter::mtState::attInd()+2) = pow(att_initStdz,2);
  for(unsigned int i=0;i<nTags;i++){
    pFilter->initStateP_(mtFilter::mtState::tagPosInd(i)+0,mtFilter::mtState::tagPosInd(i)+0) = pow(IrIT_initStdx,2);
    pFilter->initStateP_(mtFilter::mtState::tagPosInd(i)+1,mtFilter::mtState::tagPosInd(i)+1) = pow(IrIT_initStdy,2);
    pFilter->initStateP_(mtFilter::mtState::tagPosInd(i)+2,mtFilter::mtState::tagPosInd(i)+2) = pow(IrIT_initStdz,2);
    pFilter->initStateP_(mtFilter::mtState::tagAttInd(i)+0,mtFilter::mtState::tagAttInd(i)+0) = pow(qTI_initStdx,2);
    pFilter->initStateP_(mtFilter::mtState::tagAttInd(i)+1,mtFilter::mtState::tagAttInd(i)+1) = pow(qTI_initStdy,2);
    pFilter->initStateP_(mtFilter::mtState::tagAttInd(i)+2,mtFilter::mtState::tagAttInd(i)+2) = pow(qTI_initStdz,2);
  }

  double velocityRW;
  if(!ros::param::get("~IMU/VelocityRW", velocityRW)) { ROS_BREAK(); };
  double accBiasRW;
  if(!ros::param::get("~IMU/AccBiasRW", accBiasRW)) { ROS_BREAK(); };
  double attitudeRW;
  if(!ros::param::get("~IMU/AttitudeRW", attitudeRW)) { ROS_BREAK(); };
  double gyrBiasRW;
  if(!ros::param::get("~IMU/GyrBiasRW", gyrBiasRW)) { ROS_BREAK(); };
  double positionPreNoi;
  if(!ros::param::get("~States/pos/preNoi", positionPreNoi)) { ROS_BREAK(); };
  double tagPosPreNoi;
  if(!ros::param::get("~States/IrIT/preNoi", tagPosPreNoi)) { ROS_BREAK(); };
  double tagAttPreNoi;
  if(!ros::param::get("~States/qTI/preNoi", tagAttPreNoi)) { ROS_BREAK(); };

  pFilter->prenoiP_.setIdentity();
  pFilter->prenoiP_.template block<3,3>(0,0) = Eigen::Matrix3d::Identity()*pow(positionPreNoi,2);
  pFilter->prenoiP_.template block<3,3>(3,3) = Eigen::Matrix3d::Identity()*pow(velocityRW,2);
  pFilter->prenoiP_.template block<3,3>(6,6) = Eigen::Matrix3d::Identity()*pow(attitudeRW,2);
  pFilter->prenoiP_.template block<3,3>(9,9) = Eigen::Matrix3d::Identity()*pow(accBiasRW,2);
  pFilter->prenoiP_.template block<3,3>(12,12) = Eigen::Matrix3d::Identity()*pow(gyrBiasRW,2);
  for(unsigned int i=0;i<nTags;i++){
    pFilter->prenoiP_.template block<3,3>(15+3*i,15+3*i) = Eigen::Matrix3d::Identity()*pow(tagPosPreNoi,2);
    pFilter->prenoiP_.template block<3,3>(15+3*(i+nTags),15+3*(i+nTags)) = Eigen::Matrix3d::Identity()*pow(tagAttPreNoi,2);
  }
  double TagPixelStd;
  if(!ros::param::get("~Tag/PixelStd", TagPixelStd)) { ROS_BREAK(); };
  pFilter->updnoiP_.setIdentity();
  pFilter->updnoiP_ = pow(TagPixelStd,2)*pFilter->updnoiP_;
  pFilter->computeTagCorners();
}


#endif /* PARAMETERLOADER_HPP_ */
