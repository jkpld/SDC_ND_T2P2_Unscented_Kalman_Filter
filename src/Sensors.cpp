#include "Sensors.h"
#include "Eigen/Dense"
// #include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*******************************************************************************
/*   Radar class definitions
/******************************************************************************/
Radar::Radar() {
  R = MatrixXd::Zero(3,3);
  R.diagonal() << 0.09, 0.0009, 0.09;
  angleParams_ = VectorXd::Zero(3);
  angleParams_(1) = 1;
}

Radar::~Radar() {}

MatrixXd Radar::state_to_measure(const MatrixXd &SigmaPoints) {

  // Get the number of states
  int n = SigmaPoints.cols();

  // extract the needed values
  ArrayXd rx = state.row(0);
  ArrayXd ry = state.row(1);
  ArrayXd v = state.row(2);
  ArrayXd psi = state.row(3);

  // compute the velocity
  ArrayXd vx = v*psi.cos();
  ArrayXd vy = v*psi.sin();

  // compute the object's polar angle
  ArrayXd phi = ArrayXd::Zero(n);
  for (unsigned int ii=0; ii<n; i++) {
    phi(ii) = atan2(ry(ii),rx(ii));
  }

  // initialize the output matrix
  MatrixXd meas = MatrixXd(3, n);

  meas.row(0) = sqrt(rx*rx + ry*ry); // r
  meas.row(1) = phi;
  meas.row(2) = (rx*vx + ry*vy)/r; // r_dot

  return meas;
}

VectorXd Radar::measure_to_state(const VectorXd &meas) {
  float r = meas(0);
  float phi = meas(1);

  VectorXd state = VectorXd::Zero(4);
  state(0) = r*cos(phi);
  state(1) = r*sin(phi);
  return state;
}

/*******************************************************************************
/*   Lidar class definitions
/******************************************************************************/
Lidar::Lidar() {
  R = MatrixXd::Zero(2,2);
  R.diagonal() << 0.0225, 0.0225;
  angleParams_ = VectorXd::Zero(2);
}
Lidar::~Lidar() {}

VectorXd Lidar::measure_to_state(const VectorXd &meas) {
  return H_.transpose() * meas;
}

VectorXd Lidar::state_to_measure(const VectorXd &state) {
  return H_*state;
}
MatrixXd Lidar::Jacobian(const VectorXd &state) {
  return Hj_;
}
