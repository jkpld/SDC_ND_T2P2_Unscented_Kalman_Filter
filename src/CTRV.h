#ifndef CTRV_H_
#define CTRV_H_

#include "UnscentedKF.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class CTRV : public UnscentedKF {
public:
  MatrixXd ProcessModel(MatrixXd SigPnts_aug, double dt);
};

MatrixXd CTRV::ProcessModel(MatrixXd SigPnts_aug, double dt) {

}

#endif CTRV_H_
