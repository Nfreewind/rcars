#ifndef RCARS_TAGUPDATE_HPP_
#define RCARS_TAGUPDATE_HPP_


#include "FilterStates.hpp"
#include "lightweight_filtering/Update.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rcars {

/*!
 * Innovation class, contains references to subentries:
 * - cor: reprojection error. 8 Dimensional.
 */
template<typename STATE>
class TagInnovation: public LWF::State<LWF::VectorElement<8>>{
 public:
  typedef LWF::State<LWF::VectorElement<8>> Base;
  using Base::E_;
  static constexpr unsigned int _cor = 0;
  /*!
   * Constructor
   */
  TagInnovation(){
    static_assert(_cor+1==E_,"Error with indices");
    this->template getName<_cor>() = "cor";
  };
  ~TagInnovation(){};
};
template<typename STATE>
class TagUpdateMeasAuxiliary: public LWF::AuxiliaryBase<TagUpdateMeasAuxiliary<STATE>>{
 public:
  TagUpdateMeasAuxiliary(){
      tagId_ = -1;
      tagType_ = STATIC_TAG;
  };
  ~TagUpdateMeasAuxiliary(){};
  /*!
   * Tag ID of the measured Tag
   */
  int tagId_;
  /*!
   * Tag Type of the measured Tag
   */
  TagType tagType_;
  /*!
   * Relative tag position estimate. VrVT.
   */
  V3D tagPos_;
  /*!
   * Relative tag attitude estimate. qTV.
   */
  QPD tagAtt_;
};
/*!
 * Update measurement class, contains references to subentries:
 * - cor: corner measurements
 */
template<typename STATE>
class TagUpdateMeas: public LWF::State<LWF::VectorElement<8>,TagUpdateMeasAuxiliary<STATE>>{
 public:
  typedef LWF::State<LWF::VectorElement<8>,TagUpdateMeasAuxiliary<STATE>> Base;
  using Base::E_;
  static constexpr unsigned int _cor = 0;
  static constexpr unsigned int _aux = _cor+1;
  TagUpdateMeas(){
    static_assert(_aux+1==E_,"Error with indices");
  };
  ~TagUpdateMeas(){};
};
template<typename STATE>
class TagUpdateNoise: public LWF::State<LWF::VectorElement<8>>{
 public:
  typedef LWF::State<LWF::VectorElement<8>> Base;
  using Base::E_;
  static constexpr unsigned int _cor = 0;
  TagUpdateNoise(){
    static_assert(_cor+1==E_,"Error with indices");
    this->template getName<_cor>() = "cor";
  };
  ~TagUpdateNoise(){};
};
template<typename STATE>
class TagOutlierDetection: public LWF::OutlierDetection<LWF::ODEntry<TagInnovation<STATE>::template getId<TagInnovation<STATE>::_cor>(),8>>{
};

template<typename FILTERSTATE>
class TagUpdate: public LWF::Update<TagInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,TagUpdateMeas<typename FILTERSTATE::mtState>,TagUpdateNoise<typename FILTERSTATE::mtState>,
                                    TagOutlierDetection<typename FILTERSTATE::mtState>,false>{
 public:
  typedef LWF::Update<TagInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,TagUpdateMeas<typename FILTERSTATE::mtState>,TagUpdateNoise<typename FILTERSTATE::mtState>,
                      TagOutlierDetection<typename FILTERSTATE::mtState>,false> Base;
  using Base::doubleRegister_;
  using Base::intRegister_;
  using Base::updnoiP_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtFilterCovMat mtFilterCovMat;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  /*!
   * Camera matrix
   */
  Eigen::Matrix3d CameraMatrix_;
  /*!
   * Each column of this 3x4 matrix contains the vector from the tag coordinate frame to one of the 4 tag corners
   */
  Eigen::Matrix<double,3,4> TrTC_;
  /*!
   * Tags edge size
   */
  double tagSize_;
  TagUpdate(){
    tagSize_ = 0.1; // TODO: register
    computeTagCorners();
    CameraMatrix_.setIdentity();
    for(int i=0;i<8;i++){
      const int ind = mtNoise::template getId<mtNoise::_cor>()+i;
      doubleRegister_.removeScalarByVar(updnoiP_(ind,ind));
      doubleRegister_.registerScalar("PixelStd",updnoiP_(ind,ind));
    }
    intRegister_.removeScalarByStr("maxNumIteration");
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    doubleRegister_.removeScalarByStr("updateVecNormTermination");
    doubleRegister_.registerScalar("tagSize",tagSize_);
  };
  ~TagUpdate(){};
  void refreshProperties(){
    computeTagCorners();
  };
  /*!
   * Computes the various tag corner positions w.r.t. the tag coordinate frame. Depends on tagSize_.
   */
  void computeTagCorners(){
    for(unsigned int i=0;i<4;i++){
      TrTC_.col(i) = Eigen::Vector3d(0.5*tagSize_*(2*int(i%2)-1),0.5*tagSize_*(2*int(i<2)-1),0.0);
    }
  }
  /*!
   * Update model of filter. Based on corner measurement.
   * Directly evaluates the innovation term (= reporjection error)
   */
  void eval(mtInnovation& y, const mtState& state, const mtMeas& meas, const mtNoise noise, double dt = 0.0) const{
    /* Reprojection error calculation.
     * Compute position of camera in inertial frame:
     * IrIV = IrIM  + qIM*(MrMV)
     * Compute position of tag corner in inertial frame:
     * IrIC = IrIT + qTI^T*TrTC
     * Take the difference of the above values and transform into current camera frame
     * VrVC = qVM*qIM^T*(IrIC-IrIV)
     * Map to pixel coordinates by multiplying with camera matrix and projecting onto image plane
     * p = project_z(K*VrVC)
     */
    Eigen::Vector3d IrIV;
    Eigen::Vector3d IrIC;
    Eigen::Vector3d VrVC;
    Eigen::Vector3d TrTC;
    Eigen::Vector3d p;
    IrIV = state.template get<mtState::_pos>() + state.template get<mtState::_att>().rotate(state.template get<mtState::_vep>());
    y.template get<mtInnovation::_cor>().setZero();
    const int tagId = meas.template get<mtMeas::_aux>().tagId_;
    if(tagId != -1){
      if(meas.template get<mtMeas::_aux>().tagType_ == DYNAMIC_TAG){
        const int ind = state.template get<mtState::_aux>().getDynamicIndFromTagId(tagId);
        if(ind != -1){
          for(unsigned int j=0;j<4;j++){
            TrTC = TrTC_.col(j);
            IrIC = state.template get<mtState::_dyp>(ind) + state.template get<mtState::_dya>(ind).inverseRotate(TrTC);
            VrVC = (state.template get<mtState::_vea>()*state.template get<mtState::_att>().inverted()).rotate(Eigen::Vector3d(IrIC-IrIV));
            p = CameraMatrix_*VrVC;
            double z = p(2);
            p = p/z;
            y.template get<mtInnovation::_cor>()(j*2+0) = p(0)-meas.template get<mtMeas::_cor>()(j*2+0);
            y.template get<mtInnovation::_cor>()(j*2+1) = p(1)-meas.template get<mtMeas::_cor>()(j*2+1);
          }
        }
      }
    }
    y.template get<mtInnovation::_cor>() += noise.template get<mtNoise::_cor>();
  }
  /*!
   * Jacobian of update model with respect to the filter state
   */
  void jacInput(mtJacInput& F, const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    F.setZero();
    Eigen::Vector3d IrIV;
    Eigen::Vector3d IrIC;
    Eigen::Vector3d VrVC;
    Eigen::Vector3d TrTC;
    Eigen::Vector3d p;
    Eigen::Matrix3d M3; // Temporary 3d matrix
    Eigen::Matrix<double,1,3> J1; // Partial jacobian with respect to first coordinate
    Eigen::Matrix<double,1,3> J2; // Partial jacobian with respect to second coordinate
    IrIV = state.template get<mtState::_pos>() + state.template get<mtState::_att>().rotate(state.template get<mtState::_vep>());
    const int tagId = meas.template get<mtMeas::_aux>().tagId_;
    if(tagId != -1){
      if(meas.template get<mtMeas::_aux>().tagType_ == DYNAMIC_TAG){
        const int ind = state.template get<mtState::_aux>().getDynamicIndFromTagId(tagId);
        if(ind != -1){
          for(unsigned int j=0;j<4;j++){
            TrTC = TrTC_.col(j);
            IrIC = state.template get<mtState::_dyp>(ind) + state.template get<mtState::_dya>(ind).inverseRotate(TrTC);
            VrVC = (state.template get<mtState::_vea>()*state.template get<mtState::_att>().inverted()).rotate(Eigen::Vector3d(IrIC-IrIV));
            p = CameraMatrix_*VrVC;
            J1.setZero();
            J1(0,0) = 1/p(2);
            J1(0,2) = -p(0)/pow(p(2),2);
            J1 = J1*CameraMatrix_;
            J2.setZero();
            J2(0,1) = 1/p(2);
            J2(0,2) = -p(1)/pow(p(2),2);
            J2 = J2*CameraMatrix_;
            M3 = -rot::RotationMatrixPD(state.template get<mtState::_vea>()*state.template get<mtState::_att>().inverted()).matrix();
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+0,mtState::template getId<mtState::_pos>()) = J1*M3;
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+1,mtState::template getId<mtState::_pos>()) = J2*M3;
            M3 = -rot::RotationMatrixPD(state.template get<mtState::_vea>()*state.template get<mtState::_att>().inverted()).matrix()*gSM(IrIC-state.template get<mtState::_pos>());
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+0,mtState::template getId<mtState::_att>()) = J1*M3;
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+1,mtState::template getId<mtState::_att>()) = J2*M3;
            M3 = rot::RotationMatrixPD(state.template get<mtState::_vea>()*state.template get<mtState::_att>().inverted()).matrix();
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+0,mtState::template getId<mtState::_dyp>(ind)) = J1*M3;
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+1,mtState::template getId<mtState::_dyp>(ind)) = J2*M3;
            M3 = -rot::RotationMatrixPD(state.template get<mtState::_vea>()*state.template get<mtState::_att>().inverted()*state.template get<mtState::_dya>(ind).inverted()).matrix()*gSM(TrTC);
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+0,mtState::template getId<mtState::_dya>(ind)) = J1*M3;
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+1,mtState::template getId<mtState::_dya>(ind)) = J2*M3;
            M3 = gSM(VrVC);
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+0,mtState::template getId<mtState::_vea>()) = J1*M3;
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+1,mtState::template getId<mtState::_vea>()) = J2*M3;
            M3 = -rot::RotationMatrixPD(state.template get<mtState::_vea>()).matrix();
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+0,mtState::template getId<mtState::_vep>()) = J1*M3;
            F.template block<1,3>(mtInnovation::template getId<mtInnovation::_cor>()+j*2+1,mtState::template getId<mtState::_vep>()) = J2*M3;
          }
        }
      }
    }
  }
  /*!
   * Jacobian of update model with respect to the update noise
   */
  void jacNoise(mtJacNoise& G, const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    G.setIdentity();
  }

  /*!
   * This method is executed before an update.
   * It contains the handling of newly observed tags.
   */
  void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished){
    isFinished = false;
    const int tagId = meas.template get<mtMeas::_aux>().tagId_;
    if(tagId != -1){
      if(meas.template get<mtMeas::_aux>().tagType_ == DYNAMIC_TAG){
        if(filterState.state_.template get<mtState::_aux>().getDynamicIndFromTagId(tagId) == -1){
          int newInd = filterState.state_.template get<mtState::_aux>().getFreeDynamicInd(); // Check if there is still space in the filter state for a further tag
          if(newInd >= 0){
            filterState.makeNewDynamicTag(newInd,meas.template get<mtMeas::_aux>().tagId_,meas.template get<mtMeas::_aux>().tagPos_,meas.template get<mtMeas::_aux>().tagAtt_); // Add the new tag to the filter
          } else {
            std::cout << "Was not able to create new tag, maximal number of dynamic tags reached" << std::endl;
          }
        }
      }
    } else {
      isFinished = true;
    }
  };

  /*!
   * This method is executed after an update.
   */
  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    isFinished = true;
  };
};

}


#endif /* RCARS_TAGUPDATE_HPP_ */
