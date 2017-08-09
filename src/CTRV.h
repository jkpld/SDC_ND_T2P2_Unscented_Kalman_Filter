#ifndef CTRV_H_
#define CTRV_H_

#include "UKF.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* CTRV - constant turn rate and velocity
*/

class CTRV : public UKF {
public:
  CTRV();
  ~CTRV();

  // define virtual methods
  MatrixXd ProcessModel(MatrixXd SigPnts_aug, double dt);
  void labelParametersAsAngle(VectorXd& isAngle);
};

CTRV::CTRV() {
  // Set problem size
  n_x_ = 5;
  n_v_ = 2;
  double n = n_x_ + n_v_;

  // Initialize matrices and vectors;
  x_ = VectorXd::Zero(n_x);
  x_aug_ = VectorXd::Zero(n);
  P_ = MatrixXd::Zero(n_x,n_x);
  P_aug_ = MatrixXd::Zero(n, n);
  nu_ = VectorXd::Zero(n_v_);

  // Initialize what parameters are angles
  labelParametersAsAngle();

  // set the default spreading parameter value.
  setLambda(3-n);
}

void labelParametersAsAngle() {
  angleParams_ = VectorXd::Zero(n_x_);
  angleParams_(3) = 1;
}

MatrixXd CTRV::ProcessModel(MatrixXd SigPnts_aug, double dt) {

}

#endif CTRV_H_
