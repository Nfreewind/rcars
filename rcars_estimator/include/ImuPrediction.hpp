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

#ifndef RCARS_IMUPREDICTION_HPP_
#define RCARS_IMUPREDICTION_HPP_

#include "FilterStates.hpp"
#include "lightweight_filtering/Prediction.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rcars {

template<typename FILTERSTATE>
class ImuPrediction: public LWF::Prediction<FILTERSTATE>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  /*!
   * Verbose flag
   */
  bool verbose_;
  ImuPrediction():g_(0,0,-9.81){
    int ind;
    for(int i=0;i<mtState::nDynamicTags_;i++){
      ind = mtState::template getId<mtState::_dyp>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind+0,ind+0));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+1,ind+1));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+2,ind+2));
      doubleRegister_.registerScalar("PredictionNoise.dyp_0",prenoiP_(ind+0,ind+0));
      doubleRegister_.registerScalar("PredictionNoise.dyp_1",prenoiP_(ind+1,ind+1));
      doubleRegister_.registerScalar("PredictionNoise.dyp_2",prenoiP_(ind+2,ind+2));
      ind = mtState::template getId<mtState::_dya>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind+0,ind+0));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+1,ind+1));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+2,ind+2));
      doubleRegister_.registerScalar("PredictionNoise.dya_0",prenoiP_(ind+0,ind+0));
      doubleRegister_.registerScalar("PredictionNoise.dya_1",prenoiP_(ind+1,ind+1));
      doubleRegister_.registerScalar("PredictionNoise.dya_2",prenoiP_(ind+2,ind+2));

    }
    for(int i=0;i<mtState::nHybridTags_;i++){
      ind = mtState::template getId<mtState::_hya>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind+0,ind+0));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+1,ind+1));
      doubleRegister_.registerScalar("PredictionNoise.hya_0",prenoiP_(ind+0,ind+0));
      doubleRegister_.registerScalar("PredictionNoise.hya_1",prenoiP_(ind+1,ind+1));
    }
    disablePreAndPostProcessingWarning_ = true;
    verbose_ = false;
  };
  ~ImuPrediction(){};
  /*!
   * Prediction model of filter. Based on IMU measurements.
   */
  void eval(mtState& output, const mtState& state, const mtMeas& meas, const mtNoise noise, double dt) const{
    output.template get<mtState::_aux>().MwIMmeas_ = meas.template get<mtMeas::_gyr>();
    output.template get<mtState::_aux>().MwIMest_ = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    const V3D imuRor = output.template get<mtState::_aux>().MwIMest_+noise.template get<mtNoise::_att>()/sqrt(dt);
    const V3D dImuOmega = dt*imuRor;
    QPD dQ = dQ.exponentialMap(-dImuOmega);
    output.template get<mtState::_pos>() = state.template get<mtState::_pos>()-dt*(state.template get<mtState::_att>().rotate(state.template get<mtState::_vel>())
        -noise.template get<mtNoise::_pos>()/sqrt(dt));
    output.template get<mtState::_vel>() = (M3D::Identity()-gSM(dImuOmega))*state.template get<mtState::_vel>()
        -dt*(meas.template get<mtMeas::_acc>()-state.template get<mtState::_acb>()+state.template get<mtState::_att>().inverseRotate(g_)-noise.template get<mtNoise::_vel>()/sqrt(dt));
    output.template get<mtState::_acb>() = state.template get<mtState::_acb>()+noise.template get<mtNoise::_acb>()*sqrt(dt);
    output.template get<mtState::_gyb>() = state.template get<mtState::_gyb>()+noise.template get<mtNoise::_gyb>()*sqrt(dt);
    output.template get<mtState::_att>() = state.template get<mtState::_att>()*dQ;
    output.template get<mtState::_vep>() = state.template get<mtState::_vep>()+noise.template get<mtNoise::_vep>()*sqrt(dt);
    dQ = dQ.exponentialMap(noise.template get<mtNoise::_vea>()*sqrt(dt));
    output.template get<mtState::_vea>() = dQ*state.template get<mtState::_vea>();

    // Predict new robocentric tag position
    const V3D camRor = state.template get<mtState::_vea>().rotate(imuRor);
    const V3D camVel = state.template get<mtState::_vea>().rotate(V3D(imuRor.cross(state.template get<mtState::_vep>())-state.template get<mtState::_vel>()));
    for(unsigned int i=0;i<mtState::nDynamicTags_;i++){
      output.template get<mtState::_dyp>(i) = (M3D::Identity()-gSM(V3D(dt*camRor)))*state.template get<mtState::_dyp>(i)-dt*camVel+noise.template get<mtNoise::_dyp>(i)*sqrt(dt);
      dQ = dQ.exponentialMap(-dt*camRor + noise.template get<mtNoise::_dya>(i)*sqrt(dt));
      output.template get<mtState::_dya>(i) = state.template get<mtState::_dya>(i)*dQ;
    }
    for(unsigned int i=0;i<mtState::nHybridTags_;i++){
      state.template get<mtState::_hya>(i).boxPlus((noise.template get<mtNoise::_hya>(i)*sqrt(dt)).eval(),output.template get<mtState::_hya>(i));
    }

    output.template get<mtState::_aux>().wMeasCov_ = prenoiP_.template block<3,3>(mtNoise::template getId<mtNoise::_att>(),mtNoise::template getId<mtNoise::_att>())/dt;
    output.template get<mtState::_aux>().timeSinceLastValidUpdate_ = state.template get<mtState::_aux>().timeSinceLastValidUpdate_+dt;
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
    const V3D dImuOmega = dt*imuRor;
    F.setZero();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_pos>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_vel>()) = -dt*MPD(state.template get<mtState::_att>()).matrix();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_att>()) = -dt*gSM(state.template get<mtState::_att>().rotate(state.template get<mtState::_vel>()));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_vel>()) = (M3D::Identity()-gSM(dImuOmega));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_acb>()) = dt*M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_gyb>()) = -dt*gSM(state.template get<mtState::_vel>());
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_att>()) = dt*MPD(state.template get<mtState::_att>()).matrix().transpose()*gSM(g_);
    F.template block<3,3>(mtState::template getId<mtState::_acb>(),mtState::template getId<mtState::_acb>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtState::template getId<mtState::_gyb>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_gyb>()) = dt*MPD(state.template get<mtState::_att>()).matrix()*Lmat(-dImuOmega);
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_att>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_vep>(),mtState::template getId<mtState::_vep>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_vea>(),mtState::template getId<mtState::_vea>()) = M3D::Identity();

    const V3D camRor = state.template get<mtState::_vea>().rotate(imuRor);
    const V3D camVel = state.template get<mtState::_vea>().rotate(V3D(imuRor.cross(state.template get<mtState::_vep>())-state.template get<mtState::_vel>()));
    for(unsigned int i=0;i<mtState::nDynamicTags_;i++){
      F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_dyp>(i)) = (M3D::Identity()-gSM(V3D(dt*camRor)));
      F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_vel>()) = dt*MPD(state.template get<mtState::_vea>()).matrix();
      F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_gyb>()) =
          -dt*MPD(state.template get<mtState::_vea>()).matrix()*gSM(state.template get<mtState::_vep>())
          -dt*gSM(state.template get<mtState::_dyp>(i))*MPD(state.template get<mtState::_vea>()).matrix();
      F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_vep>()) =
          -dt*MPD(state.template get<mtState::_vea>()).matrix()*gSM(imuRor);
      F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_vea>()) =
          -dt*gSM(state.template get<mtState::_vea>().rotate(V3D(imuRor.cross(state.template get<mtState::_vep>())-state.template get<mtState::_vel>())))
          +dt*gSM(state.template get<mtState::_dyp>(i))*gSM(state.template get<mtState::_vea>().rotate(imuRor));
      F.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtState::template getId<mtState::_dya>(i)) = M3D::Identity();
      F.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtState::template getId<mtState::_gyb>()) =
          dt*MPD(state.template get<mtState::_dya>(i)).matrix()*Lmat(-dt*camRor)*MPD(state.template get<mtState::_vea>()).matrix();
      F.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtState::template getId<mtState::_vea>()) =
          -dt*MPD(state.template get<mtState::_dya>(i)).matrix()*Lmat(-dt*camRor)*gSM(state.template get<mtState::_vea>().rotate(imuRor));
    }
    for(unsigned int i=0;i<mtState::nHybridTags_;i++){
      F.template block<2,2>(mtState::template getId<mtState::_hya>(i),mtState::template getId<mtState::_hya>(i)).setIdentity();
    }
  }
  /*!
   * Jacobian of prediction model with respect to the process noise
   */
  void jacNoise(mtJacNoise& G, const mtState& state, const mtMeas& meas, double dt) const{
    const V3D imuRor = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    const V3D dImuOmega = dt*imuRor;
    G.setZero();
    G.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_vel>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_att>()) =
        gSM(state.template get<mtState::_vel>())*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_acb>(),mtNoise::template getId<mtNoise::_acb>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtNoise::template getId<mtNoise::_gyb>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_att>(),mtNoise::template getId<mtNoise::_att>()) =
        -MPD(state.template get<mtState::_att>()).matrix()*Lmat(-dImuOmega)*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vep>(),mtNoise::template getId<mtNoise::_vep>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vea>(),mtNoise::template getId<mtNoise::_vea>()) = M3D::Identity()*sqrt(dt);

    const V3D camRor = state.template get<mtState::_vea>().rotate(imuRor);
    for(unsigned int i=0;i<mtState::nDynamicTags_;i++){
      G.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtNoise::template getId<mtNoise::_dyp>(i)) = M3D::Identity()*sqrt(dt);
      G.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtNoise::template getId<mtNoise::_att>()) =
          MPD(state.template get<mtState::_vea>()).matrix()*gSM(state.template get<mtState::_vep>())*sqrt(dt)
          +gSM(state.template get<mtState::_dyp>(i))*MPD(state.template get<mtState::_vea>()).matrix()*sqrt(dt);
      G.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtNoise::template getId<mtNoise::_att>()) =
          -MPD(state.template get<mtState::_dya>(i)).matrix()*Lmat(-dt*camRor)*MPD(state.template get<mtState::_vea>()).matrix()*sqrt(dt);
      G.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtNoise::template getId<mtNoise::_dya>(i)) =
          MPD(state.template get<mtState::_dya>(i)).matrix()*Lmat(-dt*camRor)*sqrt(dt);
    }

    for(unsigned int i=0;i<mtState::nHybridTags_;i++){
      G.template block<2,2>(mtState::template getId<mtState::_hya>(i),mtNoise::template getId<mtNoise::_hya>(i)) = Eigen::Matrix2d::Identity()*sqrt(dt);
    }
  }
};

}


#endif /* RCARS_IMUPREDICTION_HPP_ */
