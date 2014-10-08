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

/*!
 * Coordinate frame overview:
 * - I: Inertial frame
 * - B: Body frame
 * - M: Imu-fixed coordinate frame
 * - V: Camera-fixed coordinate frame
 * - T: Tag-fixed coordinate frame
 */



#ifndef FILTERRCARS_HPP_
#define FILTERRCARS_HPP_

#include "EKF.hpp"
#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>

namespace rot = kindr::rotations::eigen_impl;

/*!
 * Filter Namespace
 */
namespace FilterRCARS {

/*!
 * State class, contains references to substates:
 * - pos: robocentric position
 * - vel: robocentric velocity
 * - att: robocentric attitude
 * - acb: accelerometer bias
 * - gyb: gyroscope bias
 * - tagPos: tag positions
 * - tagAtt: tag attitudes
 * Also includes tracking of further quantities.
 * nTags is the maximum number of tags that can be kept in the filter state.
 */
template<int nTags>
class State:public LightWeightUKF::State<0,4+nTags,1+nTags>{
 public:
  /*!
   * Constructor. Initialize additional quantities.
   */
  State(){
    for(unsigned int i=0;i<nTags;i++){
      tagId_(i) = -1;
    }
    MwIMest_.setZero();
    wMeasCov_.setIdentity();
  };
  /*!
   * Position accessors. MrIM.
   */
  const Eigen::Vector3d& pos() const{return this->v(0);}
  Eigen::Vector3d& pos() {return this->v(0);}
  /*!
   * Velocity accessors. MvM.
   */
  const Eigen::Vector3d& vel() const{return this->v(1);}
  Eigen::Vector3d& vel() {return this->v(1);}
  /*!
   * Attitude accessors. qIM.
   */
  const rot::RotationQuaternionPD& att() const{return this->q(0);}
  rot::RotationQuaternionPD& att() {return this->q(0);}
  /*!
   * Accelerometer bias accessors. Mb_acc.
   */
  const Eigen::Vector3d& acb() const{return this->v(2);}
  Eigen::Vector3d& acb() {return this->v(2);}
  /*!
   * Gyroscope bias accessors. Mb_gyr.
   */
  const Eigen::Vector3d& gyb() const{return this->v(3);}
  Eigen::Vector3d& gyb() {return this->v(3);}
  /*!
   * Tag position accessors. IrIT.
   * - i: tag index
   */
  const Eigen::Vector3d& tagPos(unsigned int i) const{
    assert(i<nTags);
    return this->v(4+i);
  }
  Eigen::Vector3d& tagPos(unsigned int i) {
    assert(i<nTags);
    return this->v(4+i);
  }
  /*!
   * Tag attitude accessors. qTI.
   * - i: tag index
   */
  const rot::RotationQuaternionPD& tagAtt(unsigned int i) const{
    assert(i<nTags);
    return this->q(1+i);
  }
  rot::RotationQuaternionPD& tagAtt(unsigned int i) {
    assert(i<nTags);
    return this->q(1+i);
  }
  /*!
   * Index functions, return the index in the covariance matrix belonging to a specific state
   */
  static unsigned int posInd(){ return 0;}
  static unsigned int velInd(){ return 3;}
  static unsigned int attInd(){ return 12+nTags*3;}
  static unsigned int acbInd(){ return 6;}
  static unsigned int gybInd(){ return 9;}
  static unsigned int tagPosInd(unsigned int i){
    return 12+i*3;
  }
  static unsigned int tagAttInd(unsigned int i){
    return 15+(nTags+i)*3;
  }
  /*!
   * Vector containing ID of tags
   * Contains -1 if not yet assigned
   */
  Eigen::Matrix<int,nTags,1> tagId_;
  /*!
   * Estimated rotational rate (bias corrected)
   */
  Eigen::Vector3d MwIMest_;
  /*!
   * Actual gyroscope covariance matrix (discretized form)
   */
  Eigen::Matrix3d wMeasCov_;
  /*!
   * Searches the ID vector for a specific tag ID and returns the vector index
   * Returns -1 if not found.
   */
  int getIndFromTagId(int tagId) const{
    for(unsigned int i=0;i<nTags;i++){
      if(tagId_(i)==tagId){
        return i;
      }
    }
    return -1;
  }
  /*!
   * Attempts to find an empty tag state.
   * Returns -1 if all tag states are occupied.
   */
  int getFreeInd() const{
    for(unsigned int i=0;i<nTags;i++){
      if(tagId_(i)==-1){
        return i;
      }
    }
    return -1;
  }
  /*!
   * Overloading boxplus operator for passing some additional quantities.
   */
  void boxplus(const typename LightWeightUKF::State<0,4+nTags,1+nTags>::DiffVec& vecIn, State<nTags>& stateOut) const{
    LightWeightUKF::State<0,4+nTags,1+nTags>::boxplus(vecIn,stateOut);
    stateOut.tagId_ = tagId_;
    stateOut.MwIMest_ = MwIMest_;
    stateOut.wMeasCov_ = wMeasCov_;
  }
};

/*!
 * Innovation class, contains references to subentries:
 * - rer: reprojection error.
 * nTags is the maximum number of tags that can be kept in the filter state.
 */
template<int nTags>
class Innovation:public LightWeightUKF::State<nTags*8,0,0>{
 public:
  /*!
   * Constructor
   */
  Innovation(){};
  /*!
   * Reprojection error accessors.
   * - i: tag index {0,1,2,...,nTags-1}
   * - j: coordinate index {0,1}
   */
  const double& rer(unsigned int i,unsigned int j) const{
    assert(i<nTags && j<8);
    return this->s(i*8+j);
  }
  double& rer(unsigned int i,unsigned int j){
    assert(i<nTags && j<8);
    return this->s(i*8+j);
  }
  /*!
   * Index functions, return the index in the covariance matrix belonging to a specific entry
   */
  static unsigned int rerInd(unsigned int i,unsigned int j){ return i*8+j;}
};

/*!
 * Prediction measurement class, contains references to subentries:
 * - acc: accelerometer measurement
 * - gyr: gyroscope measurement
 */
class PredictionMeas:public LightWeightUKF::State<0,2,0>{
 public:
  /*!
   * Constructor
   */
  PredictionMeas(){};
  /*!
   * Accelerometer measurement accessors.
   */
  const Eigen::Vector3d& acc() const{return this->v(0);}
  Eigen::Vector3d& acc() {return this->v(0);}
  /*!
   * Gyroscope measurement accessors.
   */
  const Eigen::Vector3d& gyr() const{return this->v(1);}
  Eigen::Vector3d& gyr() {return this->v(1);}
};

/*!
 * Update measurement class, contains references to subentries:
 * - cor: corner measurements
 * - tagPos: estimated relative tag position
 * - tagAtt: estimated relative tag attitude
 * nTags is the maximum number of tags that can be kept in the filter state.
 */
template<int nTags>
class UpdateMeas:public LightWeightUKF::State<nTags*8,nTags,nTags>{
 public:
  /*!
   * Constructor. Sets all tagIds to -1 (no measurements)
   */
  UpdateMeas(){
    resetInformation();
  };
  /*!
   * Vector containing ID of tags
   * Contains -1 if no measurement
   */
  Eigen::Matrix<int,nTags,1> tagId_;
  /*!
   * Corner measurement of tag
   * - i: tag index {0,1,2,...,nTags-1}
   * - j: coordinate index {0,1}
   */
  const double& cor(unsigned int i,unsigned int j) const{
    assert(i<nTags && j<8);
    return this->s(i*8+j);
  }
  double& cor(unsigned int i,unsigned int j){
    assert(i<nTags && j<8);
    return this->s(i*8+j);
  }
  /*!
   * Relative tag position estimate. VrVT.
   * - i: tag index
   */
  const Eigen::Vector3d& tagPos(unsigned int i) const{
    assert(i<nTags);
    return this->v(i);
  }
  Eigen::Vector3d& tagPos(unsigned int i) {
    assert(i<nTags);
    return this->v(i);
  }
  /*!
   * Relative tag attitude estimate. qTV.
   * - i: tag index
   */
  const rot::RotationQuaternionPD& tagAtt(unsigned int i) const{
    assert(i<nTags);
    return this->q(i);
  }
  rot::RotationQuaternionPD& tagAtt(unsigned int i) {
    assert(i<nTags);
    return this->q(i);
  }
  /*!
   * Index functions, return the index in the covariance matrix belonging to a specific state
   */
  static unsigned int corInd(unsigned int i,unsigned int j){ return i*8+j;}
  static unsigned int tagPosInd(unsigned int i){
    return nTags*8+i*3;
  }
  static unsigned int tagAttInd(unsigned int i){
    return nTags*8+(nTags+i)*3;
  }
  /*!
   * Returns true if the update measurement actually contains information, i.e., if at least one tag ID is not -1
   */
  bool hasInformation(){
    bool b = false;
    for(unsigned int i=0;i<nTags;i++){
      b = b | (tagId_(i) != -1);
    }
    return b;
  }
  /*!
   * Sets all tag IDs to -1
   */
  void resetInformation(){
    for(unsigned int i=0;i<nTags;i++){
      tagId_(i) = -1;
    }
  }
};

/*!
 * Filter class.
 * nTags is the maximum number of tags that can be kept in the filter state.
 */
template<unsigned int nTags>
class Filter:public LightWeightEKF::EKF<State<nTags>,Innovation<nTags>,PredictionMeas,UpdateMeas<nTags>,15+nTags*6,nTags*8>{
 public:
  /*!
   * Typedefs and using-declarations
   */
  typedef typename LightWeightEKF::EKF<State<nTags>,Innovation<nTags>,PredictionMeas,UpdateMeas<nTags>,15+nTags*6,nTags*8> Base;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtPredictionMeas mtPredictionMeas;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtUpdateMeas mtUpdateMeas;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  typedef typename Base::mtPredictionJacState mtPredictionJacState;
  typedef typename Base::mtPredictionJacNoise mtPredictionJacNoise;
  typedef typename Base::mtInnovationJacState mtInnovationJacState;
  typedef typename Base::mtInnovationJacNoise mtInnovationJacNoise;
  using Base::initState_;
  using Base::initStateP_;
  using Base::prenoiP_;
  using Base::updnoiP_;
  using Base::preupdnoiP_;
  using Base::state_;
  using Base::stateSafe_;
  using Base::stateFront_;
  using Base::outlierDetection_;
  using Base::D_;
  using Base::iD_;
  using Base::stateP_;
  using Base::defaultPredictionMeas_;

  /*!
   * Maximum number of tags that can be kept in the filter state.
   */
  static const unsigned int nTags_ = nTags;
  /*!
   * Rotation between body and camera
   */
  rot::RotationQuaternionPD qVB_;
  /*!
   * Translation between body and camera
   */
  Eigen::Vector3d BrBV_;
  /*!
   * Rotation between body and IMU
   */
  rot::RotationQuaternionPD qMB_;
  /*!
   * Translation between body and IMU
   */
  Eigen::Vector3d BrBM_;
  /*!
   * Threshold for outlier detection. Chi-Square distributed with 6 DOF
   */
  double mTh_;
  /*!
   * Gravity vector expressed in the inertial frame
   */
  const Eigen::Vector3d g_;
  /*!
   * Tags edge size
   */
  double tagSize_;
  /*!
   * Camera matrix
   */
  Eigen::Matrix3d CameraMatrix_;
  /*!
   * Each column of this 3x4 matrix contains the vector from the tag coordinate frame to one of the 4 tag corners
   */
  Eigen::Matrix<double,3,4> TrTC_;
  /*!
   * Constructor
   */

  Filter(): g_(0.0,0.0,-9.81){
    initState_.setIdentity();
    initStateP_.setIdentity();
    initStateP_ = initStateP_*std::pow(0.1,2);
    prenoiP_.setIdentity();
    prenoiP_ = prenoiP_*std::pow(0.1,2);
    updnoiP_.setIdentity();
    updnoiP_ = updnoiP_*std::pow(0.1,2);
    BrBM_ = Eigen::Vector3d(0.0,0.0,0.0);
    qMB_ = rot::RotationQuaternionPD(1.0,0.0,0.0,0.0);
    BrBV_ = Eigen::Vector3d(0.0,0.0,0.0);
    qVB_ = rot::RotationQuaternionPD(1.0,0.0,0.0,0.0);
    mTh_ = 7.82;
    tagSize_ = 0.1;
    CameraMatrix_.setIdentity();
    computeTagCorners();
  }

  /*!
   * Computes the various tag corner positions w.r.t. the tag coordinate frame. Depends on tagSize_.
   */
  void computeTagCorners(){
    for(unsigned int i=0;i<4;i++){
      TrTC_.col(i) = Eigen::Vector3d(0.5*tagSize_*(2*int(i%2)-1),0.5*tagSize_*(2*int(i<2)-1),0.0);
    }
  }

  /*!
   * Destructor
   */
  ~Filter(){};

  /*!
   * Prediction model of filter. Based on IMU measurements.
   */
  mtState evalPrediction(const mtState* mpState, const mtPredictionMeas* mpPredictionMeas, const double dt) const{
    mtState statePre;

    // Compute bias corrected gyroscope measurement and compute incemental rotation qQ
    statePre.MwIMest_ = mpPredictionMeas->gyr()-mpState->gyb();
    Eigen::Vector3d dOmega = -dt*statePre.MwIMest_;
    rot::RotationQuaternionPD dQ = dQ.exponentialMap(dOmega);

    // Predict the different states based on the previous state stored in mpState, the current IMU measurement in mpPredictionMeas, and the timestep dt
    statePre.att() = mpState->att()*dQ;
    statePre.att().fix(); // Re-normalize quaternion
    statePre.pos() = (Eigen::Matrix3d::Identity()+kindr::linear_algebra::getSkewMatrixFromVector(dOmega))*mpState->pos()-dt*mpState->vel();
    statePre.vel() = (Eigen::Matrix3d::Identity()+kindr::linear_algebra::getSkewMatrixFromVector(dOmega))*mpState->vel()-dt*(mpPredictionMeas->acc()-mpState->acb()+mpState->att().inverseRotate(g_));
    statePre.acb() = mpState->acb();
    statePre.gyb() = mpState->gyb();
    for(unsigned int i=0;i<nTags_;i++){
      statePre.tagPos(i) = mpState->tagPos(i);
      statePre.tagAtt(i) = mpState->tagAtt(i);
    }

    // Take care of auxilary states
    statePre.tagId_ = mpState->tagId_;
    statePre.wMeasCov_ = prenoiP_.template block<3,3>(6,6)/dt;
    return statePre;
  }

  /*!
   * Jacobian of prediction model with respect to the filter state
   */
  mtPredictionJacState evalPredictionJacState(const mtState* mpState, const mtPredictionMeas* mpPredictionMeas, const double dt) const{
    mtPredictionJacState F;
    F.setIdentity();
    Eigen::Vector3d dOmega = -dt*(mpPredictionMeas->gyr()-mpState->gyb());
    F.template block<3,3>(mtState::posInd(),mtState::posInd()) = (Eigen::Matrix3d::Identity()+kindr::linear_algebra::getSkewMatrixFromVector(dOmega));
    F.template block<3,3>(mtState::posInd(),mtState::velInd()) = -dt*Eigen::Matrix3d::Identity();
    F.template block<3,3>(mtState::posInd(),mtState::gybInd()) = -dt*kindr::linear_algebra::getSkewMatrixFromVector(mpState->pos());
    F.template block<3,3>(mtState::velInd(),mtState::velInd()) = (Eigen::Matrix3d::Identity()+kindr::linear_algebra::getSkewMatrixFromVector(dOmega));
    F.template block<3,3>(mtState::velInd(),mtState::acbInd()) = dt*Eigen::Matrix3d::Identity();
    F.template block<3,3>(mtState::velInd(),mtState::gybInd()) = -dt*kindr::linear_algebra::getSkewMatrixFromVector(mpState->vel());
    F.template block<3,3>(mtState::velInd(),mtState::attInd()) = dt*rot::RotationMatrixPD(mpState->att()).matrix().transpose()*kindr::linear_algebra::getSkewMatrixFromVector(g_);
    F.template block<3,3>(mtState::acbInd(),mtState::acbInd()) = Eigen::Matrix3d::Identity();
    F.template block<3,3>(mtState::gybInd(),mtState::gybInd()) = Eigen::Matrix3d::Identity();
    F.template block<3,3>(mtState::attInd(),mtState::gybInd()) = dt*rot::RotationMatrixPD(mpState->att()).matrix()*LightWeightEKF::Lmat(dOmega);
    F.template block<3,3>(mtState::attInd(),mtState::attInd()) = Eigen::Matrix3d::Identity();
    for(unsigned int i=0;i<nTags_;i++){
      F.template block<3,3>(mtState::tagPosInd(i),mtState::tagPosInd(i)) = Eigen::Matrix3d::Identity();
      F.template block<3,3>(mtState::tagAttInd(i),mtState::tagAttInd(i)) = Eigen::Matrix3d::Identity();
    }
    return F;
  }

  /*!
   * Jacobian of prediction model with respect to the process noise
   */
  mtPredictionJacNoise evalPredictionJacNoise(const mtState* mpState, const mtPredictionMeas* mpPredictionMeas, const double dt) const{
    mtPredictionJacNoise Fn;
    Fn.setZero();
    Eigen::Vector3d dOmega = -dt*(mpPredictionMeas->gyr()-mpState->gyb());
    Fn.template block<3,3>(mtState::posInd(),0) = Eigen::Matrix3d::Identity()*sqrt(dt);
    Fn.template block<3,3>(mtState::velInd(),3) = Eigen::Matrix3d::Identity()*sqrt(dt);
    Fn.template block<3,3>(mtState::velInd(),6) = -kindr::linear_algebra::getSkewMatrixFromVector(mpState->vel())*sqrt(dt);
    Fn.template block<3,3>(mtState::acbInd(),9) = Eigen::Matrix3d::Identity()*sqrt(dt);
    Fn.template block<3,3>(mtState::gybInd(),12) = Eigen::Matrix3d::Identity()*sqrt(dt);
    Fn.template block<3,3>(mtState::attInd(),6) = rot::RotationMatrixPD(mpState->att()).matrix()*LightWeightEKF::Lmat(dOmega)*sqrt(dt);
    for(unsigned int i=0;i<nTags_;i++){
      Fn.template block<3,3>(mtState::tagPosInd(i),15+3*i) = Eigen::Matrix3d::Identity()*sqrt(dt);
      Fn.template block<3,3>(mtState::tagAttInd(i),15+3*(i+nTags_)) = Eigen::Matrix3d::Identity()*sqrt(dt);
    }
    return Fn;
  }

  /*!
   * Update model of filter. Based on corner measurement.
   * Directly evaluates the innovation term (= reporjection error)
   */
  mtInnovation evalInnovation(const mtState* mpState, const mtPredictionMeas* mpPredictionMeas, const mtUpdateMeas* mpUpdateMeas, const double dt) const{
    mtInnovation y;

    /* Reprojection error calculation.
     * Compute position of camera in inertial frame:
     * IrIV = qIM*MrIM  + qIM*qMB*(- BrBM + BrBV)
     * Compute position of tag corner in inertial frame:
     * IrIC = IrIT + qTI^T*TrTC
     * Take the difference of the above values and transform into current camera frame
     * VrVC = qVB*qMB^T*qIM^T*(IrIC-IrIV)
     * Map to pixel coordinates by multiplying with camera matrix and projecting onto image plane
     * p = project_z(K*VrVC)
     */
    Eigen::Vector3d IrIV;
    Eigen::Vector3d TrTC;
    Eigen::Vector3d IrIC;
    Eigen::Vector3d VrVC;
    Eigen::Vector3d p;
    IrIV = mpState->att().rotate(Eigen::Vector3d(mpState->pos() + qMB_.rotate(Eigen::Vector3d(BrBV_-BrBM_))));
    // Do the above calculation for each tag corner measurement, otherwise set reprojection to zero
    for(unsigned int i=0;i<nTags_;i++){
      for(unsigned int j=0;j<8;j++){
        y.rer(i,j) = 0.0;
      }
      if(mpUpdateMeas->tagId_(i) != -1){
        int ind = mpState->getIndFromTagId(mpUpdateMeas->tagId_(i)); // Find if the tag ID is available in the state
        if(ind != -1){
          for(unsigned int j=0;j<4;j++){
            TrTC = TrTC_.col(j);
            IrIC = mpState->tagPos(ind) + mpState->tagAtt(ind).inverseRotate(TrTC);
            VrVC = (qVB_*qMB_.inverted()*mpState->att().inverted()).rotate(Eigen::Vector3d(IrIC-IrIV));
            p = CameraMatrix_*VrVC;
            double z = p(2);
            p = p/z; // Projection
            y.rer(i,j*2+0) = p(0)-mpUpdateMeas->cor(i,j*2+0);
            y.rer(i,j*2+1) = p(1)-mpUpdateMeas->cor(i,j*2+1);
          }
        }
      }
    }
    return y;
  }

  /*!
   * Jacobian of update model with respect to the filter state
   */
  mtInnovationJacState evalInnovationJacState(const mtState* mpState, const mtPredictionMeas* mpPredictionMeas, const mtUpdateMeas* mpUpdateMeas, const double dt) const{
    mtInnovationJacState H;
    H.setZero();

    // Computation of Jacobians, see evalInnovation(...)
    Eigen::Vector3d IrIV;
    Eigen::Vector3d TrTC;
    Eigen::Vector3d IrIC;
    Eigen::Vector3d VrVC;
    Eigen::Vector3d p;
    Eigen::Matrix<double,1,3> J1; // Partial jacobian with respect to first coordinate
    Eigen::Matrix<double,1,3> J2; // Partial jacobian with respect to second coordinate
    Eigen::Matrix3d M3; // Temporary 3d matrix
    IrIV = mpState->att().rotate(Eigen::Vector3d(mpState->pos() + qMB_.rotate(Eigen::Vector3d(BrBV_-BrBM_))));
    for(unsigned int i=0;i<nTags_;i++){
      if(mpUpdateMeas->tagId_(i) != -1){
        int ind = mpState->getIndFromTagId(mpUpdateMeas->tagId_(i));
        if(ind != -1){
          for(unsigned int j=0;j<4;j++){
            TrTC = TrTC_.col(j);
            IrIC = mpState->tagPos(ind) + mpState->tagAtt(ind).inverseRotate(TrTC);
            VrVC = (qVB_*qMB_.inverted()*mpState->att().inverted()).rotate(Eigen::Vector3d(IrIC-IrIV));
            p = CameraMatrix_*VrVC;
            J1.setZero();
            J1(0,0) = 1/p(2);
            J1(0,2) = -p(0)/pow(p(2),2);
            J2.setZero();
            J2(0,1) = 1/p(2);
            J2(0,2) = -p(1)/pow(p(2),2);
            M3 = -CameraMatrix_*rot::RotationMatrixPD(qVB_*qMB_.inverted()).matrix();
            H.template block<1,3>(mtInnovation::rerInd(i,j*2+0),mtState::posInd()) = J1*M3;
            H.template block<1,3>(mtInnovation::rerInd(i,j*2+1),mtState::posInd()) = J2*M3;
            M3 = -CameraMatrix_*rot::RotationMatrixPD(qVB_*qMB_.inverted()*mpState->att().inverted()).matrix()*kindr::linear_algebra::getSkewMatrixFromVector(IrIC);
            H.template block<1,3>(mtInnovation::rerInd(i,j*2+0),mtState::attInd()) = J1*M3;
            H.template block<1,3>(mtInnovation::rerInd(i,j*2+1),mtState::attInd()) = J2*M3;
            M3 = CameraMatrix_*rot::RotationMatrixPD(qVB_*qMB_.inverted()*mpState->att().inverted()).matrix();
            H.template block<1,3>(mtInnovation::rerInd(i,j*2+0),mtState::tagPosInd(ind)) = J1*M3;
            H.template block<1,3>(mtInnovation::rerInd(i,j*2+1),mtState::tagPosInd(ind)) = J2*M3;
            M3 = -CameraMatrix_*rot::RotationMatrixPD(qVB_*qMB_.inverted()*mpState->att().inverted()*mpState->tagAtt(ind).inverted()).matrix()*kindr::linear_algebra::getSkewMatrixFromVector(TrTC);
            H.template block<1,3>(mtInnovation::rerInd(i,j*2+0),mtState::tagAttInd(ind)) = J1*M3;
            H.template block<1,3>(mtInnovation::rerInd(i,j*2+1),mtState::tagAttInd(ind)) = J2*M3;
          }
        }
      }
    }
    return H;
  }

  /*!
   * Jacobian of update model with respect to the update noise
   */
  mtInnovationJacNoise evalInnovationJacNoise(const mtState* mpState, const mtPredictionMeas* mpPredictionMeas, const mtUpdateMeas* mpUpdateMeas, const double dt) const{
    mtInnovationJacNoise Hn;
    Hn.setIdentity();
    return Hn;
  }

  /*!
   * This method is executed before an update.
   * It contains the handling of newly observed tags.
   */
  void preProcess(const mtPredictionMeas* mpPredictionMeas,const mtUpdateMeas* mpUpdateMeas,const double dt){
    for(unsigned int i=0;i<nTags_;i++){ // Do for each available tag measurement which cannot be found in the current state
      if(mpUpdateMeas->tagId_(i) != -1){
        if(state_.getIndFromTagId(mpUpdateMeas->tagId_(i)) == -1){
          int newInd = state_.getFreeInd(); // Check if there is still space in the filter state for a further tag
          if(newInd >= 0){
            makeNewTag(newInd,mpUpdateMeas->tagId_(i),mpUpdateMeas->tagPos(i),mpUpdateMeas->tagAtt(i)); // Add the new tag to the filter
          } else {
            std::cout << "Was not able to create new tag, maximal number reached" << std::endl;
          }
        }
      }
    }
  }

  /*!
   * This method is executed after an update.
   */
  void postProcess(const mtPredictionMeas* mpPredictionMeas,const mtUpdateMeas* mpUpdateMeas,const double dt){
    // The following quantities are used if making predictions to times where no IMU meausurements are available.
    defaultPredictionMeas_.gyr().setZero();
    defaultPredictionMeas_.acc() = -state_.att().inverseRotate(g_);
  };

  /*!
   * Adds a new tag to the filter at index newInd and with tag ID tagId.
   * VrVT and qTV are used for initializing the tag position.
   */
  void makeNewTag(unsigned int newInd,int tagId,const Eigen::Vector3d& VrVT,const rot::RotationQuaternionPD& qTV){
    // IrIT = IrIB + qIM*qMB*(BrBV+qVB^T*VrVT)
    state_.tagPos(newInd) = get_IrIB(state_) + (state_.att()*qMB_).rotate(Eigen::Vector3d(BrBV_ + qVB_.inverseRotate(VrVT)));
    // qTI = (qIM*qMB*qVB^T*qTV)^T
    state_.tagAtt(newInd) = (state_.att()*qMB_*qVB_.inverted()*qTV).inverted();
    state_.tagId_(newInd) = tagId;
    // Reset the covariance terms associated with the new tag state
    stateP_.template block<D_,3>(0,mtState::tagPosInd(newInd)).setZero();
    stateP_.template block<3,D_>(mtState::tagPosInd(newInd),0).setZero();
    stateP_.template block<D_,3>(0,mtState::tagAttInd(newInd)).setZero();
    stateP_.template block<3,D_>(mtState::tagAttInd(newInd),0).setZero();
    stateP_.template block<3,3>(mtState::tagPosInd(newInd),mtState::tagPosInd(newInd)) = initStateP_.template block<3,3>(mtState::tagPosInd(newInd),mtState::tagPosInd(newInd));
    stateP_.template block<3,3>(mtState::tagAttInd(newInd),mtState::tagAttInd(newInd)) = initStateP_.template block<3,3>(mtState::tagAttInd(newInd),mtState::tagAttInd(newInd));
  }

  /*!
   * Enable the detection of outliers.
   */
  void enableOutlierDetection(){
    for(unsigned int i=0;i<nTags_;i++){  // check for each foot separately
      outlierDetection_.push_back(mtOutlierDetection(8*i,8*i+7,mTh_)); // Add an outlier detection entry for each tag
    }
  }

  /*!
   * Disable the detection of outliers.
   */
  void disableOutlierDetection(){
    outlierDetection_.clear(); // Delete all outlier detection entries
  }

  /*!
   * Returns the position of the body in the inertial frame
   * IrIB = qIM*(MrIM - qMB*BrBM)
   */
  Eigen::Vector3d get_IrIB(const mtState& state){
    return state.att().rotate(state.pos()) - (state.att()*qMB_).rotate(BrBM_);
  }
  Eigen::Vector3d get_IrIB_front(){
    return get_IrIB(stateFront_);
  }
  Eigen::Vector3d get_IrIB_safe(){
    return get_IrIB(stateSafe_);
  }

  /*!
   * Returns the robocentric rotational rate w.r.t. the body frame
   * BwIB = qMB^T*MwIM
   */
  Eigen::Vector3d get_BwIB(const mtState& state){
    return qMB_.inverseRotate(state.MwIMest_);
  }

  /*!
   * Returns the velocity w.r.t. the inertial frame
   * vB = qIM*(MvM + qMB*(BwIB x -BrBM))
   */
  Eigen::Vector3d get_IvB(const mtState& state){
    return state.att().rotate(Eigen::Vector3d(-state.vel() + qMB_.rotate(Eigen::Vector3d(kindr::linear_algebra::getSkewMatrixFromVector(get_BwIB(state))*-BrBM_))));
  }

  /*!
   * Returns the attitude of the body w.r.t. the inertial frame
   * qBI = (qIM*qMB)^T
   */
  rot::RotationQuaternionPD get_qBI(const mtState& state){
    return (state.att()*qMB_).inverted();
  }
  rot::RotationQuaternionPD get_qBI_front(){
    return get_qBI(stateFront_);
  }
  rot::RotationQuaternionPD get_qBI_safe(){
    return get_qBI(stateSafe_);
  }

  /*!
   * Resets the filter state with the observation of one known tag
   */
  void resetWithTagPose(Eigen::Vector3d VrVT, rot::RotationQuaternionPD qTV, int tagId){
    /*!
     * Attitude of filter:
     * qIM = qTI^T*qTV*qVB*qMB^T
     */
    initState_.att() = initState_.tagAtt(0).inverted()*qTV*qVB_*qMB_.inverted();
    /*!
     * Position of filter:
     * MrIM = qIM*(IrIT + qTI^T*qTV*(qVB*(BrBM - BrBV) - VrVT))
     */
    initState_.pos() = initState_.att().inverseRotate(Eigen::Vector3d(initState_.tagPos(0) + (initState_.tagAtt(0).inverted()*qTV).rotate(Eigen::Vector3d(qVB_.rotate(Eigen::Vector3d(BrBM_ - BrBV_)) - VrVT))));
    initState_.tagId_(0) = tagId;
    this->reset(); // call reset function
  }

  /*!
   * Resets the filter state with the help of an accelerometer measurement assuming no velocity
   */
  void resetWithAccelerometer(const Eigen::Vector3d& fMeasInit){
    /*!
     * Compute the rotation (with minimal norm) that aligns the current accelerometer measurement with gravity
     */
    Eigen::Vector3d unitZ(0,0,1); // Gravity axis
    Eigen::Vector3d axis = kindr::linear_algebra::getSkewMatrixFromVector(fMeasInit)*unitZ; // The cross product gives the rotation axis (if not parallel)
    if(axis.norm()<1e-6){ // Catch the case if the norm is to small, i.e., if both vector are near to parallel
      axis = Eigen::Vector3d(1,0,0);
    } else {
      axis.normalize();
    }
    double angle = 0.0; // Angle between both vectors
    if(fMeasInit.norm()>1e-6){ // Only if the accelerometer is not close to zero
      angle = -acos((fMeasInit.transpose()*unitZ)(0)/fMeasInit.norm());
    }
    // Initialize the current attitude based on the above derived axis and angle
    initState_.att() = rot::AngleAxisPD(angle,axis);
    this->reset(); // call reset function
  }

  /*!
   * Compute commonly used output state
   */
  void getOutput(const mtState& state,Eigen::Vector3d& IrIB,rot::RotationQuaternionPD& qIB,Eigen::Vector3d& BvB,Eigen::Vector3d& BwB){
    /*!
     * Position of body expressed in inertial frame
     */
    IrIB = get_IrIB(state); // Position of body expressed in inertial frame
    /*!
     * Compute attitude between body and inertial frame
     * qIB = qIM*qMB
     */
    qIB = state.att()*qMB_;
    /*!
     * Combute robocentric rotational rate
     */
    BwB = get_BwIB(state);
    /*!
     * Compute robocentric velocity of body frame
     * BvB = qMB^T*(MvM - qMB*(BwIB x BrBM))
     */
    BvB = -qMB_.inverted().rotate(state.vel()) - kindr::linear_algebra::getSkewMatrixFromVector(BwB)*BrBM_;
  }

  /*!
   * Compute covariance of output computed in getOutput(...)
   */
  Eigen::Matrix<double,12,12> getOutputCovariance(const mtState& state,const typename mtState::CovMat& stateP){
    /*!
     * Reduced covariance matrix.
     * Order of state: MrIM, qIM, MvM, gyb
     */
    Eigen::Matrix<double,12,12> C;

    // Extract the reduced covariance matrix based on the following list of indices
    unsigned int index[4] = {mtState::posInd(),mtState::attInd(),mtState::velInd(),mtState::gybInd()};
    for(unsigned int i=0;i<4;i++){
      for(unsigned int j=0;j<4;j++){
        C.block<3,3>(i*3,j*3) = stateP.template block<3,3>(index[i],index[j]);
      }
    }

    // Add covariance of gyroscope noise
    C.block<3,3>(9,9) = C.block<3,3>(9,9) + state.wMeasCov_;

    // Evaluate Jacobian of output transformation (derivative of getOutput(...))
    Eigen::Matrix<double,12,12> J;
    J.setZero();
    J.block<3,3>(0,0) = rot::RotationMatrixPD(state.att()).matrix();
    J.block<3,3>(0,3) = kindr::linear_algebra::getSkewMatrixFromVector(state.att().rotate(Eigen::Vector3d(state.pos()-qMB_.rotate(BrBM_))));
    J.block<3,3>(3,3) = Eigen::Matrix3d::Identity();
    J.block<3,3>(6,6) = -rot::RotationMatrixPD(qMB_.inverted()).matrix();
    J.block<3,3>(6,9) = kindr::linear_algebra::getSkewMatrixFromVector(BrBM_)*rot::RotationMatrixPD(qMB_.inverted()).matrix();
    J.block<3,3>(9,9) = rot::RotationMatrixPD(qMB_.inverted()).matrix();

    // Compute and return transformed covariance matrix
    C = J*C*J.transpose();
    return C;
  }
};

}


#endif /* FILTERRCARS_HPP_ */
