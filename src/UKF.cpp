#include "UKF.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd
using Eigen::VectorXd

void UKF::setLambda(double lambda) {
  double n = n_x_ + n_v_;
  Lambda_ = sqrt(lambda + n);

  if (n > 0) {
    weights_ = VectorXd::Ones(2*n+1)/(2*(lambda + n))
    weights(0) *= 2*lambda;
  } else {
    weights_ = VectorXd::Zero(2*n+1);
  }
}
double UKF::getLambda() {
  return Lambda_*Lambda_ - n_x_ - n_v_;
}

void UKF::setProcessNoise(VectorXd& nu) {
  if (nu.size() != n_v_) {
    std::cout<<"Process noise vector does not have the expected length, ";
    std::cout<<n_v_;
    std::cout<<".\n";
  } else {
    nu_ << nu;

    // Generate process noise covariance
    MatrixXd Q_ = MatrixXd::Zero(n_v_, n_v_);
    Q_.diagonal() = nu;

    // Set noise block of the augmented covariance matrix
    P_aug_.block(n_x_, n_x_, n_v_, n_v_) << Q_;
  }
}

VectorXd UKF::getProcessNoise() {
  return nu_;
}

void UKF::Init(VectorXd& x, MatrixXd& P) {
  // Initialize state and covariance;
  x_ << x;
  P_ << P;
}

MatrixXd UKF::ComputeSigmaPoints() {
  // Update augmented state and covariance
  x_aug_.head(n_x_) << x_;
  P_aug_.block(0,0,n_x_,n_x_) << P_;

  // Compute the sigma points
  SigPnts_ = MatrixXd::Zero(n_x, 2*n + 1); // init matrix

  MatrixXd A_ = P_aug_.llt().matrixL(); // Sqrt P
  SigPnts_ << x, x + Lambda_ * A_, x - Lambda_ * A_;

  return SigPnts_;
}

void NormalizeAngles(MatrixXd& M, const VectorXd angleParams) {
  for (unsigned int ii=0; ii < angleParams.size(); ii++) {
    if (angleParams(ii)) {
      ArrayXd A = M.row(ii);

      // Wrap angles to (-pi, +pi]
      A = fmod(A + M_PI, 2*M_PI);
      if (A < 0) A += 2*M_PI;
      A -= M_PI;

      M.row(ii) = A;
    }
  }
}

MatrixXd ComputeStateDifference(MatrixXd& SigPnts, VectorXd& state, VectorXd& angleParams) {
  // 1. Get the difference between the sigma points and the mean sigma point
  MatrixXd diff = SigPnts.colwise() - state;
  // 2. Normalize angles
  NormalizeAngles(diff, angleParams)

  return diff;
}

MatrixXd ComputeStateDifference(MatrixXd& SigPnts, VectorXd& state) {
  // 1. Get the difference between the sigma points and the mean sigma point
  MatrixXd diff = SigPnts.colwise() - state;
  return diff;
}

void UKF::ComputeMeanAndCovariance(MatrixXd& SigmaPoints, VectorXd& mean, MatrixXd& covar, VectorXd& angleParams) {
  // Compute mean of predicted state
  mean = SigmaPoints*weights_;

  // Compute covariance of predicted state
  MatrixXd diff = ComputeStateDifference(SigmaPoints, mean, angleParams); // mean center
  covar = diff * weights_.asDiagonal() * diff.transpose();
}

void UKF::ComputeMeanAndCovariance(MatrixXd& SigmaPoints, VectorXd& mean, MatrixXd& covar) {
  // Compute mean of predicted state
  mean = SigmaPoints*weights_;

  // Compute covariance of predicted state
  MatrixXd diff = ComputeStateDifference(SigmaPoints, mean); // mean center
  covar = diff * weights_.asDiagonal() * diff.transpose();
}

void UKF::Predict(double dt) {

  // compute the sigma points
  SigPnts_in = ComputeSigmaPoints();

  // pass the sigma points through the ProcessModel()
  SigPnts_ = ProcessModel(SigPnts_in, dt);

  // Compute mean of predicted state
  ComputeMeanAndCovariance(SigPnts_, x_, P_, angleParams_)
}
