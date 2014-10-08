/*!
* @file   LeggedStateEstimatorManager.cpp
* @author   Michael Blösch
* @date   10.04.2014
 */
#include "LeggedStateEstimatorManager.hpp"
namespace rot = kindr::rotations::eigen_impl;

LeggedStateEstimatorManager::LeggedStateEstimatorManager(){
  reset();
  setUKFParameter(1e-3,2.0,0.0);
  enableOutlierDetection();
}

LeggedStateEstimatorManager::~LeggedStateEstimatorManager(){}

Eigen::Vector3d LeggedStateEstimatorManager::legKin(Eigen::Vector3d angles,unsigned int legId) const{
  const double bx = 0.2525;
  const double by = 0.185;
  const double lH = -0.0685;
  const double lT = -0.2;
  const double lS = -0.235;

  Eigen::Vector3d s;
  s(0) = ((legId<2)*2-1)*bx+lT*sin(angles(1))+lS*sin(angles(1)+angles(2));
  s(1) = -(((int)legId%2)*2-1)*by-sin(angles(0))*(lH+lT*cos(angles(1))+lS*cos(angles(1)+angles(2)));
  s(2) = cos(angles(0))*(lH+lT*cos(angles(1))+lS*cos(angles(1)+angles(2)));
  return s;
}

Eigen::Matrix3d LeggedStateEstimatorManager::legKinJac(Eigen::Vector3d angles,unsigned int legId) const{
  const double lH = -0.0685;
  const double lT = -0.2;
  const double lS = -0.235;

  Eigen::Matrix<double,3,3> J;
  J.setZero();
  J(0,1) = lS*cos(angles(1)+angles(2))+lT*cos(angles(1));
  J(0,2) = lS*cos(angles(1)+angles(2));
  J(1,0) = -cos(angles(0))*(lH+lT*cos(angles(1))+lS*cos(angles(1)+angles(2)));
  J(1,1) = sin(angles(0))*(lT*sin(angles(1))+lS*sin(angles(1)+angles(2)));
  J(1,2) = lS*sin(angles(0))*sin(angles(1)+angles(2));
  J(2,0) = -sin(angles(0))*(lH+lT*cos(angles(1))+lS*cos(angles(1)+angles(2)));
  J(2,1) = -cos(angles(0))*(lT*sin(angles(1))+lS*sin(angles(1)+angles(2)));
  J(2,2) = -lS*cos(angles(0))*sin(angles(1)+angles(2));
  return J;
}

void LeggedStateEstimatorManager::predictFilterOnly(Eigen::Vector3d accelerometerMeas, Eigen::Vector3d gyroscopeMeas, const double& elapsedTime){
  mtPredictionMeas predictionMeas;
  predictionMeas.v(0) = accelerometerMeas;
  predictionMeas.v(1) = gyroscopeMeas;
  predict(&predictionMeas,elapsedTime);
}

void LeggedStateEstimatorManager::predictAndUpdateFilter(const Eigen::Vector3d& accelerometerMeas, const Eigen::Vector3d& gyroscopeMeas, const Eigen::Matrix<double,12,2>& encoderMeas, const Eigen::Vector4i& contactFlagMeas, const double& elapsedTime){
  mtPredictionMeas predictionMeas;
  predictionMeas.v(0) = accelerometerMeas;
  predictionMeas.v(1) = gyroscopeMeas;
  mtUpdateMeas updateMeas;
  for(unsigned int i=0;i<4;i++){
    updateMeas.s(i) = contactFlagMeas(i);
    updateMeas.v(i) = encoderMeas.block<3,1>(3*i,0);
    updateMeas.v(4+i) = encoderMeas.block<3,1>(3*i,1);

  }
  predictAndUpdate(&predictionMeas,&updateMeas,elapsedTime);
}

Eigen::Vector3d LeggedStateEstimatorManager::getPosition() const{
  return state_.v(0);
}
Eigen::Vector3d LeggedStateEstimatorManager::getVelocity() const{
  return -state_.v(1);
}
rot::RotationQuaternionPD LeggedStateEstimatorManager::getAttitude() const{
  return state_.q(0).inverted();
}
Eigen::Vector3d LeggedStateEstimatorManager::getAccelerometerBias() const{
  return state_.v(2);
}
Eigen::Vector3d LeggedStateEstimatorManager::getGyroscopeBias() const{
  return state_.v(3);
}
void LeggedStateEstimatorManager::setPosition(const Eigen::Vector3d& r){
  state_.v(0) = r;
}
void LeggedStateEstimatorManager::setVelocity(const Eigen::Vector3d& v){
  state_.v(1) = -v;
}
void LeggedStateEstimatorManager::setAttitude(const rot::RotationQuaternionPD& q){
  state_.q(0) = q.inverted();
}
void LeggedStateEstimatorManager::setAccelerometerBias(const Eigen::Vector3d& c){
  state_.v(2) = c;
}
void LeggedStateEstimatorManager::setGyroscopeBias(const Eigen::Vector3d& d){
  state_.v(3) = d;
}




