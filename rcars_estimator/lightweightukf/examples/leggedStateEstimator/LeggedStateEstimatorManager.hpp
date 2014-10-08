/*!
* @file   LeggedStateEstimatorManager.cpp
* @author   Michael Blösch
* @date   10.04.2014
 */
#include "FilterVel.hpp"
#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
namespace rot = kindr::rotations::eigen_impl;

/*! \brief Class for managing the filter
 *  \class LeggedStateEstimatorManager
 *  Derived from FilterVel, takes care of the data management
 */
class LeggedStateEstimatorManager: private FilterVel{
 public:
  /*! \brief Constructor
   */
  LeggedStateEstimatorManager();

  /*! \brief Destructor
   */
  ~LeggedStateEstimatorManager();

  /*! \brief Forward kinematics
   * \param angles  Joint angles
   * \param legId  Index of leg
   * \return  Vector between body and foot expressed in body CF
   */
  Eigen::Vector3d legKin(Eigen::Vector3d angles,unsigned int legId) const;

  /*! \brief Jacobian of forward kinematics
   * \param angles  Joint angles
   * \param legId  Index of leg
   * \return  Jacobian of legKin(...)
   */
  Eigen::Matrix3d legKinJac(Eigen::Vector3d angles,unsigned int legId) const;

  /*! \brief Predict the state of the filter based on the current accelerometer and gyroscope measurements
   * \param accelerometerMeas  Accelerometer measurements
   * \param gyroscopeMeas  Gyroscope measurements
   * \param elapsedTime  Elapsed time
   */
  void predictFilterOnly(Eigen::Vector3d accelerometerMeas, Eigen::Vector3d gyroscopeMeas, const double& elapsedTime);

  /*! \brief Predict and updates the state of the filter based on the current accelerometer and gyroscope measurements as well as on the kinematic measurements
   * \param accelerometerMeas  Accelerometer measurements
   * \param gyroscopeMeas  Gyroscope measurements
   * \param encoderMeas  Encoder measurements
   * \param ContactFlagMeas  Contact flags
   * \param elapsedTime  Elapsed time
   */
  void predictAndUpdateFilter(const Eigen::Vector3d& accelerometerMeas, const Eigen::Vector3d& gyroscopeMeas, const Eigen::Matrix<double,12,2>& encoderMeas, const Eigen::Vector4i& contactFlagMeas, const double& elapsedTime);

  /*! \brief Returns the estimated position (expressed in the World frame)
   * \return  Current estimate of the position
   */
  Eigen::Vector3d getPosition() const;

  /*! \brief Returns the estimated velocity (expressed in the Body frame)
   * \return  Current estimate of the velocity
   */
  Eigen::Vector3d getVelocity() const;

  /*! \brief Returns the estimated attitude (quaternion from World frame to Body frame)
   * \return  Current estimate of the attitude
   */
  rot::RotationQuaternionPD getAttitude() const;

  /*! \brief Returns the estimated accelerometer bias (expressed in the Body frame)
   * \return  Current estimate of the accelerometer bias
   */
  Eigen::Vector3d getAccelerometerBias() const;

  /*! \brief Returns the estimated gyroscope bias (expressed in the Body frame)
   * \return  Current estimate of the gyroscope bias
   */
  Eigen::Vector3d getGyroscopeBias() const;

  /*! \brief Sets the current estimated position to the desired value
   * \param  r Position (expressed in the World frame)
   */
  void setPosition(const Eigen::Vector3d& r);

  /*! \brief Sets the current estimated velocity to the desired value
   * \param  v Velocity (expressed in the Body frame)
   */
  void setVelocity(const Eigen::Vector3d& v);

  /*! \brief Sets the current estimated attitude to the desired value
   * \param  q Attitude (quaternion from World frame to Body frame)
   */
  void setAttitude(const rot::RotationQuaternionPD& q);

  /*! \brief Sets the current estimated accelerometer bias to the desired value
   * \param  c Accelerometer bias (expressed in the Body frame)
   */
  void setAccelerometerBias(const Eigen::Vector3d& c);

  /*! \brief Sets the current estimated gyroscope bias to the desired value
   * \param  d Gyroscope bias (expressed in the Body frame)
   */
  void setGyroscopeBias(const Eigen::Vector3d& d);
};
