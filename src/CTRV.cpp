#include "CTRV.h"
#include "Eigen/Dense"

using Eigen::MatrixXd
using Eigen::VectorXd

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

MatrixXd CTRV::ProcessModel(MatrixXd& SP, double dt) {
  /* Remember: SP (Sigma points) is agumented so that the columns are
      x = [r_x, r_y, v, psi, psi_dot, nu_a, nu_psidd]
  */

  // Extract the needed quantities
  ArrayXd v = SP.row(2);
  ArrayXd psi = SP.row(3);
  ArrayXd psid = SP.row(4);
  ArrayXd nu_a = SP.row(5);
  ArrayXd nu_psidd = SP.row(6);

  ArrayXd cosPsi = psi.cos();
  ArrayXd sinPsi = psi.sin();

  // Create the noise matrix
  MatrixXd noise = MatrixXd::Zero(n_x_, SP.cols());

  noise.row(0) = 0.5*dt*cosPsi*nu_a;
  noise.row(1) = 0.5*dt*sinPsi*nu_a;
  noise.row(2) = nu_a;
  noise.row(3) = 0.5*dt*nu_psidd;
  noise.row(4) = nu_psidd;
  noise *= dt;

  // Create the predicted change in the state
  MatrixXd dx = MatrixXd::Zero(n_x_, SP.cols());

  ArrayXd isPsiZero = psid < 0.001;

  // If psi_dot is zero, then use the limit of the expression as psi_dot goes to
  //  zero
  dx.row(0) = isPsiZero.select(v*cosPsi*dt,
    (v/nu_psidd)*((psi+psid*dt).sin() - sinPsi));

  dx.row(1) = isPsiZero.select(v*sinPsi*dt,
    (v/nu_psidd)*(cosPsi - (psi+psid*dt).cos()));

  dx.row(3) = psid*dt;


  // Create the final result
  MatrixXd xk1 = dx + noise;
  return xk1;
}
