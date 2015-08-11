#ifndef RCARS_FILTERSTATES_HPP_
#define RCARS_FILTERSTATES_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rcars {

enum TagType{
  DYNAMIC_TAG,
  HYBRID_TAG,
  STATIC_TAG
};

template<unsigned int nDynamicTags, unsigned int nHybridTags>
class StateAuxiliary: public LWF::AuxiliaryBase<StateAuxiliary<nDynamicTags,nHybridTags>>{
 public:
  /*!
   * Constructor
   */
  StateAuxiliary(){
    MwIMest_.setZero();
    MwIMmeas_.setZero();
    wMeasCov_.setIdentity();
    for(unsigned int i=0;i<nDynamicTags;i++){
      dynamicIds_[i] = -1;
    }
    for(unsigned int i=0;i<nHybridTags;i++){
      hybridIds_[i] = -1;
    }
  };
  ~StateAuxiliary(){};
  /*!
   * Estimated rotational rate (bias corrected)
   */
  V3D MwIMest_;
  /*!
   * Measured rotational rate
   */
  V3D MwIMmeas_;
  /*!
   * Actual gyroscope covariance matrix (discretized form)
   */
  M3D wMeasCov_;
  /*!
   * Array containing ID of dynamic tags
   * Contains -1 if not yet assigned
   */
  int dynamicIds_[nDynamicTags];
  /*!
   * Array containing ID of hybrid tags
   * Contains -1 if not yet assigned
   */
  int hybridIds_[nHybridTags];
  /*!
   * Searches the dynamic tag ID vector for a specific tag ID and returns the vector index
   * Returns -1 if not found.
   */
  int getDynamicIndFromTagId(int tagId) const{
    for(unsigned int i=0;i<nDynamicTags;i++){
      if(dynamicIds_[i]==tagId){
        return i;
      }
    }
    return -1;
  }
  /*!
   * Attempts to find an empty dynamic tag state.
   * Returns -1 if all tag states are occupied.
   */
  int getFreeDynamicInd() const{
    for(unsigned int i=0;i<nDynamicTags;i++){
      if(dynamicIds_[i]==-1){
        return i;
      }
    }
    return -1;
  }
  /*!
   * Searches the hybrid tag ID vector for a specific tag ID and returns the vector index
   * Returns -1 if not found.
   */
  int getHybridIndFromTagId(int tagId) const{
    for(unsigned int i=0;i<nHybridTags;i++){
      if(hybridIds_[i]==tagId){
        return i;
      }
    }
    return -1;
  }
  /*!
   * Attempts to find an empty hybrid tag state.
   * Returns -1 if all tag states are occupied.
   */
  int getFreeHybridInd() const{
    for(unsigned int i=0;i<nHybridTags;i++){
      if(hybridIds_[i]==-1){
        return i;
      }
    }
    return -1;
  }
};

/*!
 * State class, contains references to substates:
 * - pos: robocentric position
 * - vel: robocentric velocity
 * - att: robocentric attitude
 * - acb: accelerometer bias
 * - gyb: gyroscope bias
 * - vep: Vison extrinsics position
 * - vea: Vison extrinsics attitude
 * - dyp: dynamic tag positions
 * - dyp: dynamic tag attitudes
 * - ĥya: hybrid tag normal
 * Also includes tracking of further quantities.
 * nDynamicTags is the maximum number of dynamic tags that can be kept in the filter state.
 * nHybridTags is the maximum number of hybrid tags that can be kept in the filter state.
 */
template<int nDynamicTags, int nHybridTags>
class State: public LWF::State<
    LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
    LWF::TH_multiple_elements<LWF::QuaternionElement,2>,
    LWF::ArrayElement<LWF::VectorElement<3>,nDynamicTags>,
    LWF::ArrayElement<LWF::QuaternionElement,nDynamicTags>,
    LWF::ArrayElement<LWF::NormalVectorElement,nHybridTags>,
    StateAuxiliary<nDynamicTags,nHybridTags>>{
 public:
  typedef LWF::State<
      LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
      LWF::TH_multiple_elements<LWF::QuaternionElement,2>,
      LWF::ArrayElement<LWF::VectorElement<3>,nDynamicTags>,
      LWF::ArrayElement<LWF::QuaternionElement,nDynamicTags>,
      LWF::ArrayElement<LWF::NormalVectorElement,nHybridTags>,
      StateAuxiliary<nDynamicTags,nHybridTags>> Base;
  using Base::D_;
  using Base::E_;
  static constexpr int nDynamicTags_ = nDynamicTags;
  static constexpr int nHybridTags_ = nHybridTags;
  static constexpr int nTags_ = nDynamicTags + nHybridTags;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _acb = _vel+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _vep = _gyb+1;
  static constexpr unsigned int _att = _vep+1;
  static constexpr unsigned int _vea = _att+1;
  static constexpr unsigned int _dyp = _vea+(int)(nDynamicTags>0);
  static constexpr unsigned int _dya = _dyp+(int)(nDynamicTags>0);
  static constexpr unsigned int _hya = _dya+(int)(nHybridTags>0);
  static constexpr unsigned int _aux = _hya+1;
  /*!
   * Constructor
   */
  State(){
    static_assert(_aux+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_vep>() = "vep";
    this->template getName<_att>() = "att";
    this->template getName<_vea>() = "vea";
    if((int)(nDynamicTags>0)) this->template getName<_dyp>() = "dyp";
    if((int)(nDynamicTags>0)) this->template getName<_dya>() = "dya";
    if((int)(nHybridTags>0)) this->template getName<_hya>() = "hya";
    this->template getName<_aux>() = "aux";
  }
  ~State(){};
};
/*!
 * Prediction measurement class, contains references to subentries:
 * - acc: accelerometer measurement
 * - gyr: gyroscope measurement
 */
class PredictionMeas: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
  static constexpr unsigned int _acc = 0;
  static constexpr unsigned int _gyr = _acc+1;
  /*!
   * Constructor
   */
  PredictionMeas(){
    static_assert(_gyr+1==E_,"Error with indices");
    this->template getName<_acc>() = "acc";
    this->template getName<_gyr>() = "gyr";
  }
  ~PredictionMeas(){};
};

template<typename STATE>
class PredictionNoise: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,7>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nDynamicTags_>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nDynamicTags_>,
      LWF::ArrayElement<LWF::VectorElement<2>,STATE::nHybridTags_>>{
 public:
  using LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,7>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nDynamicTags_>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nDynamicTags_>,
      LWF::ArrayElement<LWF::VectorElement<2>,STATE::nHybridTags_>>::E_;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _acb = _vel+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _vep = _gyb+1;
  static constexpr unsigned int _att = _vep+1;
  static constexpr unsigned int _vea = _att+1;
  static constexpr unsigned int _dyp = _vea+(int)(STATE::nDynamicTags_>0);
  static constexpr unsigned int _dya = _dyp+(int)(STATE::nDynamicTags_>0);
  static constexpr unsigned int _hya = _dya+(int)(STATE::nHybridTags_>0);
  /*!
   * Constructor
   */
  PredictionNoise(){
    static_assert(_hya+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_vep>() = "vep";
    this->template getName<_att>() = "att";
    this->template getName<_vea>() = "vea";
    if((int)(STATE::nDynamicTags_>0)) this->template getName<_dyp>() = "dyp";
    if((int)(STATE::nDynamicTags_>0)) this->template getName<_dya>() = "dya";
    if((int)(STATE::nHybridTags_>0)) this->template getName<_hya>() = "hya";
  }
  ~PredictionNoise(){};
};

template<int nDynamicTags, int nHybridTags>
class FilterState: public LWF::FilterState<State<nDynamicTags,nHybridTags>,PredictionMeas,PredictionNoise<State<nDynamicTags,nHybridTags>>,0,true>{
 public:
  typedef LWF::FilterState<State<nDynamicTags,nHybridTags>,PredictionMeas,PredictionNoise<State<nDynamicTags,nHybridTags>>,0,true> Base;
  typedef typename Base::mtState mtState;
  using Base::state_;
  using Base::cov_;
  using Base::usePredictionMerge_;
  Eigen::Matrix<double,6,6> dynamicTagInitCov_; // TODO register
  /*!
   * Constructor
   */
  FilterState(){
    dynamicTagInitCov_.setIdentity();
  }

  /*!
   * Adds a new dynamic tag to the filter at index newInd and with tag ID tagId.
   * VrVT and qTV are used for initializing the tag position.
   */
  void makeNewDynamicTag(unsigned int newInd,int tagId,const Eigen::Vector3d& VrVT,const rot::RotationQuaternionPD& qTV){
    // IrIT = IrIB + qIM*(MrMV+qVM^T*VrVT)
    state_.template get<mtState::_dyp>(newInd) = state_.template get<mtState::_pos>()
        + state_.template get<mtState::_att>().rotate((state_.template get<mtState::_vep>()+state_.template get<mtState::_vea>().inverseRotate(VrVT)).eval());
    // qTI = (qIM*qVM^T*qTV^T)^T
    state_.template get<mtState::_dya>(newInd) = (state_.template get<mtState::_att>()*state_.template get<mtState::_vea>().inverted()*qTV.inverted()).inverted();
    state_.template get<mtState::_aux>().dynamicIds_[newInd] = tagId;
    // Reset the covariance terms associated with the new tag state
    cov_.template block<mtState::D_,3>(0,mtState::template getId<mtState::_dyp>(newInd)).setZero();
    cov_.template block<3,mtState::D_>(mtState::template getId<mtState::_dyp>(newInd),0).setZero();
    cov_.template block<mtState::D_,3>(0,mtState::template getId<mtState::_dya>(newInd)).setZero();
    cov_.template block<3,mtState::D_>(mtState::template getId<mtState::_dya>(newInd),0).setZero();
    cov_.template block<3,3>(mtState::template getId<mtState::_dyp>(newInd),mtState::template getId<mtState::_dyp>(newInd)) = dynamicTagInitCov_.template block<3,3>(0,0);
    cov_.template block<3,3>(mtState::template getId<mtState::_dyp>(newInd),mtState::template getId<mtState::_dya>(newInd)) = dynamicTagInitCov_.template block<3,3>(0,3);
    cov_.template block<3,3>(mtState::template getId<mtState::_dya>(newInd),mtState::template getId<mtState::_dyp>(newInd)) = dynamicTagInitCov_.template block<3,3>(3,0);
    cov_.template block<3,3>(mtState::template getId<mtState::_dya>(newInd),mtState::template getId<mtState::_dya>(newInd)) = dynamicTagInitCov_.template block<3,3>(3,3);
  }
//  void initWithImuPose(V3D IrIM, QPD qMI){ // TODO
//    state_.template get<mtState::_pos>() = qMI.rotate(IrIM);
//    state_.template get<mtState::_att>() = qMI.inverted();
//  }
  void initWithAccelerometer(const V3D& fMeasInit){
  V3D unitZ(0,0,1);
  if(fMeasInit.norm()>1e-6){
    state_.template get<mtState::_att>().setFromVectors(unitZ,fMeasInit);
  } else {
    state_.template get<mtState::_att>().setIdentity();
  }
}
//  void initializeFeatureState(unsigned int i, V3D n, double d,const Eigen::Matrix<double,3,3>& initCov){
//    state_.template get<mtState::_dep>(i) = d;
//    state_.template get<mtState::_nor>(i).setFromVector(n);
//    cov_.template block<mtState::D_,1>(0,mtState::template getId<mtState::_dep>(i)).setZero();
//    cov_.template block<1,mtState::D_>(mtState::template getId<mtState::_dep>(i),0).setZero();
//    cov_.template block<mtState::D_,2>(0,mtState::template getId<mtState::_nor>(i)).setZero();
//    cov_.template block<2,mtState::D_>(mtState::template getId<mtState::_nor>(i),0).setZero();
//    cov_.template block<1,1>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_dep>(i)) = initCov.block<1,1>(0,0);
//    cov_.template block<1,2>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_nor>(i)) = initCov.block<1,2>(0,1);
//    cov_.template block<2,1>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_dep>(i)) = initCov.block<2,1>(1,0);
//    cov_.template block<2,2>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_nor>(i)) = initCov.block<2,2>(1,1);
//  }
//  void removeFeature(unsigned int i){
//    state_.template get<mtState::_dep>(i) = 1.0;
//    state_.template get<mtState::_nor>(i).setIdentity();
//    cov_.template block<mtState::D_,1>(0,mtState::template getId<mtState::_dep>(i)).setZero();
//    cov_.template block<1,mtState::D_>(mtState::template getId<mtState::_dep>(i),0).setZero();
//    cov_.template block<mtState::D_,2>(0,mtState::template getId<mtState::_nor>(i)).setZero();
//    cov_.template block<2,mtState::D_>(mtState::template getId<mtState::_nor>(i),0).setZero();
//    cov_.template block<1,1>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_dep>(i)).setIdentity();
//    cov_.template block<2,2>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_nor>(i)).setIdentity();
//  }
};

}


#endif /* RCARS_FILTERSTATES_HPP_ */
