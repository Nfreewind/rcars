/*!
* @file 	FilterVel.hpp
* @author 	Michael Blösch
* @date		10.04.2014
* @brief	Filter for legged state estimation based on foot velocity
 */

#ifndef FILTERVEL_HPP_
#define FILTERVEL_HPP_

#include "UKF.hpp"
#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>

namespace rot = kindr::rotations::eigen_impl;

/*! \brief Class for fusing kinematics and inertial measurements
 *  \class FilterVel
 *  Derived from LightWeightUKF. Supports slippage detection and offset between kinematics and IMU coordinate frame. Has two purely virtual function which need to be defined in derived class.
 */
class FilterVel:public LightWeightUKF::UKF<LightWeightUKF::State<0,4,1>,LightWeightUKF::State<0,4,0>,LightWeightUKF::State<0,2,0>,LightWeightUKF::State<4,8,0>,15,12>{
 public:

  /*! \brief Constructor
   */
  FilterVel();

  /*! \brief Destructor
   */
  ~FilterVel();

  /*! \brief Prediction equation
   * \param mpState  Pointer to previous state
   * \param mpPredictionMeas  Pointer to prediction measurement
   * \param mtProcessNoise  Pointer to process noise vector
   * \param dt  Elapsed time
   * \return  Predicted state
   */
  mtState evalPrediction(const mtState* mpState, const mtPredictionMeas* mpPredictionMeas, const mtProcessNoise pNoise, const double dt) const;

  /*! \brief Update equation
   * \param mpState  Pointer to previous state
   * \param mpPredictionMeas  Pointer to prediction measurement
   * \param mpPredictionMeas  Pointer to update measurement
   * \param mtProcessNoise  Pointer to process noise vector
   * \param mtProcessNoise  Pointer to update noise vector
   * \param dt  Elapsed time
   * \return  Innovation term
   */
  mtInnovation evalInnovation(const mtState* mpState, const mtPredictionMeas* mpPredictionMeas, const mtUpdateMeas* mpUpdateMeas, const mtProcessNoise pNoise,const mtUpdateNoise uNoise, const double dt) const;

  /*! \brief Enables outlier detection
   */
  void enableOutlierDetection();

  /*! \brief Disables outlier detection
   */
  void disableOutlierDetection();

  /*! \brief Forward kinematics
   * \param angles  Joint angles
   * \param legId  Index of leg
   * \return  Vector between body and foot expressed in body CF
   */
  virtual Eigen::Vector3d legKin(Eigen::Vector3d angles,unsigned int legId) const = 0;

  /*! \brief Jacobian of forward kinematics
   * \param angles  Joint angles
   * \param legId  Index of leg
   * \return  Jacobian of legKin(...)
   */
  virtual Eigen::Matrix3d legKinJac(Eigen::Vector3d angles,unsigned int legId) const = 0;

  /*! \brief Rotation from body to IMU CF
   */
  rot::RotationQuaternionPD q_MB_;

  /*! \brief Vector between body and IMU expressed in body CF
   */
  Eigen::Vector3d B_r_BM_;

  /*! \brief Gravitational acceleration expressed in inertial CF
   */
  const Eigen::Vector3d g_;

  /*! \brief Mahalanobis threshold for slippage detection
   */
  double mTh_;
};

#endif /* FILTERVEL_HPP_ */
