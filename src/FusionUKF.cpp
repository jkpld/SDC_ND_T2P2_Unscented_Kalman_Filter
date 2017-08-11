#include "FusionUKF.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Constructor.
FusionUKF::FusionUKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  VectorXd noise = VectorXd::Zero(ctrv_.n_v_);
  noise << 0.2, 0.1;
  ctrv_.setProcessNoise(noise);
}

// Destructor.
FusionUKF::~FusionUKF() {}

void FusionUKF::ProcessMeasurement(MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // convert from measurment space to state space.
    VectorXd x_ = VectorXd::Zero(4);
    switch (measurement_pack.sensor_type_) {
     case MeasurementPackage::RADAR:
       x_ = radar_.measure_to_state(measurement_pack.raw_measurements_);
       break;
     case MeasurementPackage::LASER:
       x_ = lidar_.measure_to_state(measurement_pack.raw_measurements_);
       break;
    }

    // initialize state covariance matrix P
  	MatrixXd P_ = MatrixXd::Identity(ctrv_.n_x_, ctrv_.n_x_);
    P_.diagonal() << 0.2, 0.2, 1, 0.6, 3;

    ctrv_.Init(x_, P_);

    // initialize time stamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Update F and Q using dt
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ctrv_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ctrv_.Update(measurement_pack.raw_measurements_, radar_);
  } else {
    // Laser updates
    ctrv_.Update(measurement_pack.raw_measurements_, lidar_);
  }
}
