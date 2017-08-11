#include "UKF.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::ArrayXd;

void UKF::Init(VectorXd& x, MatrixXd& P) {
  // Initialize state and covariance;
  x_ << x;
  P_ << P;
}

void UKF::setLambda(double lambda) {
  double n = n_x_ + n_v_;
  Lambda_ = sqrt(lambda + n);

  if (n > 0) {
    weights_ = VectorXd::Ones(2*n+1)/(2*(lambda + n));
    weights_(0) *= 2*lambda;
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

void UKF::Predict(double dt) {

  // compute the sigma points
  MatrixXd SigPnts = ComputeSigmaPoints();

  // pass the sigma points through the ProcessModel() and save the result to
  // the property SigPnts_ for later use.
  SigPnts_ << ProcessModel(SigPnts, dt);

  // Compute mean of predicted state
  ComputeMeanAndCovariance(SigPnts_, x_, P_, angleParams_);
}

void UKF::Update(VectorXd &zk1, Sensor &sensor) {

  // Compute the predicted measurment from the current state (sigma points)
  //  1. Convert the state to measurment space.
  MatrixXd Z = sensor.state_to_measure(SigPnts_); // nz x L

  //  2. Compute the mean and covariance of the predicted sigma points.
  VectorXd zk1k = VectorXd::Zero(sensor.n); // nz x 1
  MatrixXd Sk1k = MatrixXd::Zero(sensor.n, sensor.n); // nz x nz
  ComputeMeanAndCovariance(Z, zk1k, Sk1k, sensor.angleParams_);

  //  3. Add in the sensor noise.
  Sk1k += sensor.R;

  // Compute cross-correlation
  MatrixXd dX = ComputeStateDifference(SigPnts_, x_, angleParams_); // nx x L
  MatrixXd dZ = ComputeStateDifference(Z, zk1k, sensor.angleParams_); // nz x L

  MatrixXd T = dX * weights_.asDiagonal() * dZ.transpose(); // (nx x nz) = (nx x L) (L x L) (L x nz)

  // Compute Kalman Gain
  MatrixXd Sk1k_inv = Sk1k.inverse();
  MatrixXd K = T * Sk1k_inv; // nx x nz

  // Compute predicted and measurment difference
  VectorXd dz = zk1 - zk1k;
  NormalizeAngles(dz, sensor.angleParams_); // nz x 1

  // Compute updated state and covariance
  x_ += K * dz; // nx x 1
  P_ -= K * Sk1k * K.transpose(); // nx x nx

  // Compute NIS
  NIS_ = dz.transpose() * Sk1k_inv * dz;
}

MatrixXd UKF::ComputeSigmaPoints() {

  // Update augmented state and covariance
  x_aug_.head(n_x_) << x_;
  P_aug_.block(0,0,n_x_,n_x_) << P_;

  // Compute the augmented sigma points
  double n = n_x_ + n_v_;
  MatrixXd SigPnts = MatrixXd::Zero(n, 2*n + 1); // init matrix

  MatrixXd A_ = P_aug_.llt().matrixL(); // Sqrt P
  SigPnts << x_aug_, (Lambda_ * A_).colwise()+x_aug_, (-Lambda_ * A_).colwise()+x_aug_;

  return SigPnts;
}

void UKF::ComputeMeanAndCovariance(MatrixXd& SigmaPoints, VectorXd& mean,
  MatrixXd& covar, VectorXd& angleParams) {

  // Compute mean of predicted state
  mean = SigmaPoints*weights_;

  // Compute covariance of predicted state
  MatrixXd diff = ComputeStateDifference(SigmaPoints, mean, angleParams); // mean center
  covar = diff * weights_.asDiagonal() * diff.transpose();
}

void UKF::ComputeMeanAndCovariance(MatrixXd& SigmaPoints, VectorXd& mean,
  MatrixXd& covar) {

  // Compute mean of predicted state
  mean = SigmaPoints*weights_;

  // Compute covariance of predicted state
  MatrixXd diff = ComputeStateDifference(SigmaPoints, mean); // mean center
  covar = diff * weights_.asDiagonal() * diff.transpose();
}

MatrixXd UKF::ComputeStateDifference(MatrixXd& SigPnts, VectorXd& state,
  VectorXd& angleParams) {

  // 1. Get the difference between the sigma points and the mean sigma point
  MatrixXd diff = SigPnts.colwise() - state;
  // 2. Normalize angles
  NormalizeAngles(diff, angleParams);
  return diff;
}

MatrixXd UKF::ComputeStateDifference(MatrixXd& SigPnts, VectorXd& state) {
  // 1. Get the difference between the sigma points and the mean sigma point
  MatrixXd diff = SigPnts.colwise() - state;
  return diff;
}

void UKF::NormalizeAngles(MatrixXd& M, VectorXd& angleParams) {
  for (unsigned int ii=0; ii < angleParams.size(); ii++) {
    if (angleParams(ii)) {
      ArrayXd A = M.row(ii);

      // Wrap angles to (-pi, +pi]
      A = A.unaryExpr([](const double x) { return fmod(x+M_PI,2*M_PI); });
      A = (A < 0).select(A+2*M_PI,A);
      // A = fmod(A + M_PI, 2*M_PI);
      // if (A < 0) A += 2*M_PI;
      A -= M_PI;

      M.row(ii) = A;
    }
  }
}

void UKF::NormalizeAngles(VectorXd& V, VectorXd& angleParams) {
  for (unsigned int ii=0; ii < angleParams.size(); ii++) {
    if (angleParams(ii)) {
      double A = V(ii);

      // Wrap angles to (-pi, +pi]
      A = fmod(A + M_PI, 2*M_PI);
      if (A < 0) A += 2*M_PI;
      A -= M_PI;

      V(ii) = A;
    }
  }
}
