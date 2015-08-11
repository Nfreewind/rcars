#ifndef RCARS_IMUPREDICTION_HPP_
#define RCARS_IMUPREDICTION_HPP_

#include "FilterStates.hpp"
#include "lightweight_filtering/Prediction.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rcars {

template<typename FILTERSTATE>
class ImuPrediction: public LWF::Prediction<FILTERSTATE>{
 public:
  typedef LWF::Prediction<FILTERSTATE> Base;
  using Base::eval;
  using Base::prenoiP_;
  using Base::doubleRegister_;
  using Base::boolRegister_;
  using Base::disablePreAndPostProcessingWarning_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  /*!
   * Gravity vector expressed in the inertial frame
   */
  const V3D g_;
  ImuPrediction():g_(0,0,-9.81){
//    int ind; //TODO
//    for(int i=0;i<mtState::nMax_;i++){
//      ind = mtNoise::template getId<mtNoise::_nor>(i);
//      doubleRegister_.removeScalarByVar(prenoiP_(ind,ind));
//      doubleRegister_.removeScalarByVar(prenoiP_(ind+1,ind+1));
//      doubleRegister_.registerScalar("PredictionNoise.nor",prenoiP_(ind,ind));
//      doubleRegister_.registerScalar("PredictionNoise.nor",prenoiP_(ind+1,ind+1));
//      ind = mtNoise::template getId<mtNoise::_dep>(i);
//      doubleRegister_.removeScalarByVar(prenoiP_(ind,ind));
//      doubleRegister_.registerScalar("PredictionNoise.dep",prenoiP_(ind,ind));
//    }
    disablePreAndPostProcessingWarning_ = true;
  };
  ~ImuPrediction(){};
  /*!
   * Prediction model of filter. Based on IMU measurements.
   */
  void eval(mtState& output, const mtState& state, const mtMeas& meas, const mtNoise noise, double dt) const{
    output.template get<mtState::_aux>().MwIMmeas_ = meas.template get<mtMeas::_gyr>();
    output.template get<mtState::_aux>().MwIMest_ = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    const V3D imuRor = output.template get<mtState::_aux>().MwIMest_+noise.template get<mtNoise::_att>()/sqrt(dt);
    const V3D dOmega = dt*imuRor;
    QPD dQ = dQ.exponentialMap(-dOmega);
    output.template get<mtState::_pos>() = state.template get<mtState::_pos>()-dt*(state.template get<mtState::_att>().rotate(state.template get<mtState::_vel>())
        -noise.template get<mtNoise::_pos>()/sqrt(dt));
    output.template get<mtState::_vel>() = (M3D::Identity()-gSM(dOmega))*state.template get<mtState::_vel>()
        -dt*(meas.template get<mtMeas::_acc>()-state.template get<mtState::_acb>()+state.template get<mtState::_att>().inverseRotate(g_)-noise.template get<mtNoise::_vel>()/sqrt(dt));
    output.template get<mtState::_acb>() = state.template get<mtState::_acb>()+noise.template get<mtNoise::_acb>()*sqrt(dt);
    output.template get<mtState::_gyb>() = state.template get<mtState::_gyb>()+noise.template get<mtNoise::_gyb>()*sqrt(dt);
    output.template get<mtState::_att>() = state.template get<mtState::_att>()*dQ;
    output.template get<mtState::_vep>() = state.template get<mtState::_vep>()+noise.template get<mtNoise::_vep>()*sqrt(dt);
    dQ = dQ.exponentialMap(noise.template get<mtNoise::_vea>()*sqrt(dt));
    output.template get<mtState::_vea>() = dQ*state.template get<mtState::_vea>();

    for(unsigned int i=0;i<mtState::nDynamicTags_;i++){
      output.template get<mtState::_dyp>(i) = state.template get<mtState::_dyp>(i)+noise.template get<mtNoise::_dyp>(i)*sqrt(dt);
      dQ = dQ.exponentialMap(noise.template get<mtNoise::_dya>(i)*sqrt(dt));
      output.template get<mtState::_dya>(i) = dQ*state.template get<mtState::_dya>(i);
    }

    for(unsigned int i=0;i<mtState::nHybridTags_;i++){
      state.template get<mtState::_hya>(i).boxPlus((noise.template get<mtNoise::_hya>(i)*sqrt(dt)).eval(),output.template get<mtState::_hya>(i));
    }
    output.template get<mtState::_aux>().wMeasCov_ = prenoiP_.template block<3,3>(mtNoise::template getId<mtNoise::_att>(),mtNoise::template getId<mtNoise::_att>())/dt;
    output.fix();
  }
  void noMeasCase(mtFilterState& filterState, mtMeas& meas, double dt){
    meas.template get<mtMeas::_gyr>() = filterState.state_.template get<mtState::_gyb>();
    meas.template get<mtMeas::_acc>() = filterState.state_.template get<mtState::_acb>()-filterState.state_.template get<mtState::_att>().inverseRotate(g_);
  }
  /*!
   * Jacobian of prediction model with respect to the filter state
   */
  void jacInput(mtJacInput& F, const mtState& state, const mtMeas& meas, double dt) const{
    const V3D imuRor = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    const V3D dOmega = dt*imuRor;
    F.setZero();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_pos>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_vel>()) = -dt*MPD(state.template get<mtState::_att>()).matrix();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_att>()) = -dt*gSM(state.template get<mtState::_att>().rotate(state.template get<mtState::_vel>()));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_vel>()) = (M3D::Identity()-gSM(dOmega));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_acb>()) = dt*M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_gyb>()) = -dt*gSM(state.template get<mtState::_vel>());
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_att>()) = dt*MPD(state.template get<mtState::_att>()).matrix().transpose()*gSM(g_);
    F.template block<3,3>(mtState::template getId<mtState::_acb>(),mtState::template getId<mtState::_acb>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtState::template getId<mtState::_gyb>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_gyb>()) = dt*MPD(state.template get<mtState::_att>()).matrix()*Lmat(-dOmega);
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_att>()) = M3D::Identity();

    for(unsigned int i=0;i<mtState::nDynamicTags_;i++){
      F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_dyp>(i)) = M3D::Identity();
      F.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtState::template getId<mtState::_dya>(i)) = M3D::Identity();
    }

    for(unsigned int i=0;i<mtState::nHybridTags_;i++){
      F.template block<2,2>(mtState::template getId<mtState::_hya>(i),mtState::template getId<mtState::_hya>(i)).setIdentity();
    }

    F.template block<3,3>(mtState::template getId<mtState::_vep>(),mtState::template getId<mtState::_vep>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_vea>(),mtState::template getId<mtState::_vea>()) = M3D::Identity();
  }
  /*!
   * Jacobian of prediction model with respect to the process noise
   */
  void jacNoise(mtJacNoise& G, const mtState& state, const mtMeas& meas, double dt) const{
    const V3D imuRor = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    const V3D dOmega = dt*imuRor;
    G.setZero();
    G.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_vel>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_att>()) =
        gSM(state.template get<mtState::_vel>())*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_acb>(),mtNoise::template getId<mtNoise::_acb>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtNoise::template getId<mtNoise::_gyb>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_att>(),mtNoise::template getId<mtNoise::_att>()) =
        -MPD(state.template get<mtState::_att>()).matrix()*Lmat(-dOmega)*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vep>(),mtNoise::template getId<mtNoise::_vep>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vea>(),mtNoise::template getId<mtNoise::_vea>()) = M3D::Identity()*sqrt(dt);

    for(unsigned int i=0;i<mtState::nDynamicTags_;i++){
      G.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtNoise::template getId<mtNoise::_dyp>(i)) = M3D::Identity()*sqrt(dt);
      G.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtNoise::template getId<mtNoise::_dya>(i)) = M3D::Identity()*sqrt(dt);
    }

    for(unsigned int i=0;i<mtState::nHybridTags_;i++){
      G.template block<2,2>(mtState::template getId<mtState::_hya>(i),mtNoise::template getId<mtNoise::_hya>(i)) = Eigen::Matrix2d::Identity()*sqrt(dt);
    }
  }
};

}


#endif /* RCARS_IMUPREDICTION_HPP_ */
