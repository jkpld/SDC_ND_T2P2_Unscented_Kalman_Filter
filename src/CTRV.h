#ifndef CTRV_H_
#define CTRV_H_

#include "UKF.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* CTRV - constant turn rate and velocity

State vector :
x = [r_x, r_y, v, psi, psi_dot]

*/

class CTRV : public UKF {
public:
  CTRV();
  ~CTRV();

  // define virtual methods
  MatrixXd ProcessModel(MatrixXd SigPnts_aug, double dt);
  void labelParametersAsAngle(VectorXd& isAngle);

};

#endif CTRV_H_
