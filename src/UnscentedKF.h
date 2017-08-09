#ifndef UnscentedKF_H_
#define UnscentedKF_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UnscentedKF {
public:

  VectorXd x_;
  MatrixXd P_;
  VectorXd isParamAngle_ = (VectorXd(1) << 999999).finished();

  // virtual method for a predict function
  virtual MatrixXd ProcessModel(MatrixXd SigPnts_aug, double dt) = 0;

  /**
   * Init initializes the uncented kalman filter
   * @param x Initial state
   * @param P Initial state covariance
   * @param nu process noise vector
   * @param lambda (optional) lambda for computed sigma points
   * @param isParamAngle (optional) A VectorXd containig the indices of the
      state parameters that are angels.
   */
  void Init(VectorXd& x, MatrixXd& P, VectorXd& nu, double lambda = 999999);

  /**
   * Method to set/get the spreading parameter.
   */
  void setLambda(double lambda);
  double getLambda();

  // Method to preform prediction;
  // @param dt Amount of time we want to predict our state into the future.
  MatrixXd Predict(double dt);

private:

  double n_x_ = 0;
  double n_v_ = 0;
  // VectorXd nu_;
  double lambda_ = 0; // spreading parameter
  double Lambda_ = sqrt(3); // sqrt(lambda + n)

  VectorXd weights_;
  VectorXd x_aug_;
  MatrixXd P_aug_;
  // MatrixXd Q_;
  MatrixXd SigPnts_;
  // MatrixXd A_;

  void ComputeSigmaPoints();
};



#endif
