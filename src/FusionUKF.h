#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "Sensors.h"

class FusionEKF {
public:
  // Constructor.
  FusionEKF();

  // Destructor.
  virtual ~FusionEKF();

  // Run the whole flow of the Kalman Filter from here.
  void ProcessMeasurement(MeasurementPackage &measurement_pack);

  /* State transition matrix and process covariance matrix
   * @param dt Time difference since last update
  */
  inline Eigen::MatrixXd F(float dt);
  inline Eigen::MatrixXd Q(float dt);

  // Kalman Filter update and prediction math lives in here.
  KalmanFilter ekf_;

  // Sensors
  Radar radar_;
  Lidar lidar_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;
};

#endif /* FusionEKF_H_ */
