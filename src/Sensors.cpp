#include "Sensors.h"
#include "Eigen/Dense"
// #include <math.h>
// #include <iostream>

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
  n = 3;
}

Radar::~Radar() {}

MatrixXd Radar::state_to_measure(const MatrixXd &SigmaPoints) {

  // Get the number of states
  int nSP = SigmaPoints.cols();

  // extract the needed values
  ArrayXd rx = state.row(0);
  ArrayXd ry = state.row(1);
  ArrayXd v = state.row(2);
  ArrayXd psi = state.row(3);

  // compute the velocity
  ArrayXd vx = v*psi.cos();
  ArrayXd vy = v*psi.sin();

  // compute the object's polar angle
  ArrayXd phi = ArrayXd::Zero(nSP);
  for (unsigned int ii=0; ii<nSP; i++) {
    phi(ii) = atan2(ry(ii),rx(ii));
  }

  // initialize the output matrix
  MatrixXd meas = MatrixXd(n, nSP);

  meas.row(0) = sqrt(rx*rx + ry*ry); // r
  meas.row(1) = phi;
  meas.row(2) = (rx*vx + ry*vy)/r; // r_dot

  return meas;
}

VectorXd Radar::measure_to_state(const VectorXd &meas) {
  float r = meas(0);
  float phi = meas(1);

  VectorXd state = VectorXd::Zero(5);
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
  n = 2;
}
Lidar::~Lidar() {}

MatrixXd Lidar::state_to_measure(const MatrixXd &SigmaPoints) {
  MatrixXd meas = MatrixXd::Zero(n,SigmaPoints.cols());
  meas.row(0) = SigmaPoints.row(0);
  meas.row(1) = SigmaPoints.row(1);
  return meas;
}

VectorXd Lidar::measure_to_state(const VectorXd &meas) {
  VectorXd state = VectorXd::Zero(5);
  state(0) = meas(0);
  state(1) = meas(1);
  return state;
}
