#ifndef UKF_H_
#define UKF_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  VectorXd x_;
  MatrixXd P_;

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

  // Method to preform prediction;
  // @param dt Amount of time we want to predict our state into the future.
  MatrixXd Predict(double dt);

private:

  double n_x_ = 0;
  double n_v_ = 0;
  double Lambda_ = sqrt(3); // sqrt(lambda + n)

  VectorXd nu_;
  VectorXd angleParams_;

  VectorXd weights_;
  VectorXd x_aug_;
  MatrixXd P_aug_;
  // MatrixXd Q_;
  MatrixXd SigPnts_;
  // MatrixXd A_;

  /*
   * ComputeSigmaPoints Method to compute the augmented sigma points from the
   *  current state/covariance
   */
  MatrixXd ComputeSigmaPoints();

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
  virtual MatrixXd ProcessModel(MatrixXd SigPnts_aug, double dt) = 0;

  void NormalizeAngles(MatrixXd& M, VectorXd angleParams);
  void ComputeMeanAndCovariance(MatrixXd& SigmaPoints, VectorXd& x, MatrixXd P);
  void ComputeMeanAndCovariance(MatrixXd& SigmaPoints, VectorXd& x, MatrixXd P, VectorXd& angleParams);
  MatrixXd ComputeStateDifference(MatrixXd& SigmaPoints, VectorXd& state);
  MatrixXd ComputeStateDifference(MatrixXd& SigmaPoints, VectorXd& state, VectorXd& angleParams);
};



#endif
