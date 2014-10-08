/*!
* @file 	FilterVel.cpp
* @author 	Michael Blösch
* @date		10.04.2014
 */

#include "FilterVel.hpp"

FilterVel::FilterVel(): g_(0.0,0.0,-9.81){
  q_MB_.setIdentity();
  B_r_BM_.setZero();
  mTh_ = 9.21;
  initState_.setIdentity();
  initStateP_.setIdentity();
  useAlternativeUpdate_ = false;
}

FilterVel::~FilterVel(){
}

FilterVel::mtState FilterVel::evalPrediction(const mtState* mpState, const mtPredictionMeas* mpPredictionMeas, const mtProcessNoise pNoise, const double dt) const{
  FilterVel::mtState statePre;
  Eigen::Vector3d dOmega = -dt*(mpPredictionMeas->v(1)-mpState->v(3)+pNoise.block<3,1>(6,0)/sqrt(dt));
  rot::RotationQuaternionPD dQ;
  dQ = dQ.exponentialMap(dOmega);
  statePre.q(0) = mpState->q(0)*dQ;
  statePre.q(0).fix();
  statePre.v(0) = mpState->v(0)-dt*mpState->q(0).rotate(mpState->v(1))+pNoise.block<3,1>(0,0)*sqrt(dt);
  statePre.v(1) = (Eigen::Matrix3d::Identity()+kindr::linear_algebra::getSkewMatrixFromVector(dOmega))*mpState->v(1)-dt*(mpPredictionMeas->v(0)-mpState->v(2)+mpState->q(0).inverseRotate(g_))+pNoise.block<3,1>(3,0)*sqrt(dt);
  statePre.v(2) = mpState->v(2)+pNoise.block<3,1>(9,0)*sqrt(dt);
  statePre.v(3) = mpState->v(3)+pNoise.block<3,1>(12,0)*sqrt(dt);
  return statePre;
}

FilterVel::mtInnovation FilterVel::evalInnovation(const mtState* mpState, const mtPredictionMeas* mpPredictionMeas, const mtUpdateMeas* mpUpdateMeas, const mtProcessNoise pNoise,const mtUpdateNoise uNoise, const double dt) const{
  mtInnovation y;
  Eigen::Vector3d M_w = mpPredictionMeas->v(1)-mpState->v(3)+pNoise.block<3,1>(6,0)/sqrt(dt);
  Eigen::Vector3d M_s;
  Eigen::Matrix3d M_J;
  for(int i=0;i<4;i++){
    M_s = q_MB_.rotate(Eigen::Vector3d(-B_r_BM_+legKin(mpUpdateMeas->v(i),i)));
    M_J = rot::RotationMatrixPD(q_MB_).matrix()*legKinJac(mpUpdateMeas->v(i),i);
    if(mpUpdateMeas->s(i)){
      y.v(i) = -mpState->v(1) + kindr::linear_algebra::getSkewMatrixFromVector(M_w)*M_s + M_J*mpUpdateMeas->v(4+i) + uNoise.block<3,1>(i*3,0);
    } else {
      y.v(i) = uNoise.block<3,1>(i*3,0);
    }
  }
  return y;
}

void FilterVel::enableOutlierDetection(){
  for(unsigned int i=0;i<4;i++){
    outlierDetection_.push_back(mtOutlierDetection(3*i,3*i+2,mTh_));
  }
}

void FilterVel::disableOutlierDetection(){
  outlierDetection_.clear();
}



