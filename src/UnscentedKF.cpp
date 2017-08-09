#include "UnscentedKF.h"
#include "Eigen/Dense"

using Eigen::MatrixXd
using Eigen::VectorXd

void UnscentedKF::setLambda(double lambda) {
  double n = n_x_ + n_v_;
  lambda_ = lambda;
  Lambda_ = sqrt(lambda_ + n);

  if (n > 0) {
    weights_ = VectorXd::Ones(2*n+1)/(2*(lambda_ + n))
    weights(0) *= 2*lambda_;
  } else {
    weights_ = VectorXd::Zero(2*n+1);
  }
}
double UnscentedKF::getLambda() {
  return lambda_;
}

void UnscentedKF::Init(VectorXd& x, MatrixXd& P, VectorXd& nu,
  double lambda = 999999) {

  /* Note, this function does not perform much error checking, such
    as if the dimensions of the inputs correctly correspond to each
    other and make sense.
  */

  // Initialize state and noise;
  x_ = x;
  P_ = P;
  // nu_ = nu;

  // Get problem sizes
  n_x_ = x.size();
  n_v_ = nu.size();
  double n = n_x_ + n_v_;

  // Compute offset factor, sqrt(lambda - n)
  if (lambda == 999999) {
    setLambda(3-n);
  } else {
    setLambda(lambda);
  }

  // Generate process noise covariance
  MatrixXd Q_ = MatrixXd::Zero(n_v_, n_v_);
  Q_.diagonal() = nu;

  // Create augmented state vector
  x_aug_ = VectorXd::Zero(n);

  // Create augmented covariance matrix
  P_aug_ = MatrixXd::Zero(n, n);
  // Initialize noise block of the augmented covariance matrix
  P_aug_.block(n_x_, n_x_, n_v_, n_v_) << Q_;

  // Create storage for sigma points matrix
  SigPnts_aug = MatrixXd::Zero(n, 2*n + 1);
  SigPnts_ = MatrixXd::Zero(n_x, 2*n + 1);
}

void UnscentedKF::ComputeSigmaPoints() {
  /**************************************************/
  /* Update augmented state and covariance */
  /**************************************************/
  x_aug_.head(n_x_) << x_;
  P_aug_.block(0,0,n_x_,n_x_) << P_;

  /**************************************************/
  /* Compute the sigma points                       */
  /**************************************************/
  MatrixXd A_ = P_aug_.llt().matrixL(); // Sqrt P
  SigPnts_aug << x, x + Lambda_ * A_, x - Lambda_ * A_;
}

void UnscentedKF::Predict(double dt) {

  // compute the sigma points
  SigPnts_in = ComputeSigmaPoints();

  // pass the sigma points through the ProcessModel()
  SigPnts_out = ProcessModel(SigPnts_in, dt);

  // Compute mean of predicted state
  x_ = SigPnts_out*weights_;

  // Compute covariance of predicted state

  // 1. Get the difference between the sigma points and the mean sigma point
  MatrixXd diff = SigPnts_out.colwise() - x_;

  // 2. Wrap any angle parameters to (-pi, +pi]
  if (isParamAngle_(0) != 999999) {
    /* 999999 is the default value; if isParamAngle_(0) is 999999, then
      isParamAngle_ was not set. In this case, assume parameters are not angles.
    */
    for (unsigned int ii=0; ii < isParamAngle_.size(); ii++) {
      double A = diff(isParamAngle_(ii));
      A = fmod(A + M_PI, 2*M_PI);
      if (A < 0)
        A += 2*M_PI;
      A -= M_PI;

      diff(isParamAngle_(ii)) = A;
    }
  }

  // 3. compute the covariance.
  P_ = diff * weights_.asDiagonal() * diff.transpose();
}
