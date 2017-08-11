#ifndef UKF_H_
#define UKF_H_

#include "Eigen/Dense"
#include "Sensors.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  VectorXd x_;
  MatrixXd P_;
  double n_x_ = 0;
  double n_v_ = 0;
  double NIS_ = 0;

  VectorXd nu_;
  VectorXd x_aug_;
  MatrixXd P_aug_;
  MatrixXd SigPnts_;
  VectorXd angleParams_;

  /**
   * Init initializes the unscented kalman filter
   * @param x Initial state
   * @param P Initial state covariance
   */
  void Init(VectorXd& x, MatrixXd& P);

  /**
   * Method to set/get the spreading parameter.
   */
  void setLambda(double lambda);
  double getLambda();

  /*
   * Method to set/get the process noise
   */
  void setProcessNoise(VectorXd& nu);
  VectorXd getProcessNoise();

  /*
   * Predict Method to preform prediction
   * @param dt Amount of time we want to predict our state into the future.
   */
  void Predict(double dt);

  /*
   * Update Method to preform update
   * @param meas New measurement.
   * @param sensor The sensor that obtained the measurment.
   */
  void Update(VectorXd &meas, Sensor &sensor);

private:


  double Lambda_ = sqrt(3); // sqrt(lambda + n)
  VectorXd weights_;

  // MatrixXd A_;

  /*
   * labelParametersAsAngle Virtual method to label state parameters as being
   *  angles
   */
  virtual void labelParametersAsAngle() = 0;

  /*
   * ProcessModel Virtual method that applies the process model to the sigma
   *  points.
   * @param SigPnts_aug The augmented sigma points
   * @param dt Amount of time we want to predict our state into the future.
   */
  virtual MatrixXd ProcessModel(MatrixXd& SigPnts_aug, double dt) = 0;

  /*
   * ComputeSigmaPoints Method to compute the augmented sigma points from the
   *  current state/covariance
   */
  MatrixXd ComputeSigmaPoints();

  /*
   * ComputeMeanAndCovariance Method for computing the mean and covariance from
      a set of sigma points
   * @param (input) SigmaPoints Matrix of the sigma points
   * @param (output) x Mean of sigma points
   * @param (output) P Covariance of sigma points
   * @param (optional, input) angleParams Vector that labels state parameters as
      angles.
   */
  void ComputeMeanAndCovariance(MatrixXd& SigmaPoints, VectorXd& x, MatrixXd& P);
  void ComputeMeanAndCovariance(MatrixXd& SigmaPoints, VectorXd& x, MatrixXd& P, VectorXd& angleParams);

  /*
   * ComputeStateDifference Method for computing the difference between two
      state vectors.
   * @param SigmaPoints Matrix where each column is a state vector
   * @param state State vector to be subtracted from each column of SigmaPoints
   * @param (optional) angleParams Vector that labels state parameters as
      angles.
   */
  MatrixXd ComputeStateDifference(MatrixXd& SigmaPoints, VectorXd& state);
  MatrixXd ComputeStateDifference(MatrixXd& SigmaPoints, VectorXd& state, VectorXd& angleParams);

  /*
   * NormalizeAngles Method to normalize angles to the range (-pi, +pi]
   * @param (in/out) M Matrix of state vectors (as columns)
   * @param angleParams Vector labeling each row of M as being an angle or not.
   */
  void NormalizeAngles(MatrixXd& M, VectorXd& angleParams);
  void NormalizeAngles(VectorXd& V, VectorXd& angleParams);
};



#endif
