#ifndef FusionUKF_H_
#define FusionUKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include "CTRV.h"
#include "Sensors.h"

class FusionUKF {
public:
  // Constructor.
  FusionUKF();

  // Destructor.
  virtual ~FusionUKF();

  // Run the whole flow of the Kalman Filter from here.
  void ProcessMeasurement(MeasurementPackage &measurement_pack);

  // Kalman Filter update and prediction math lives in here.
  CTRV ctrv_;

  // Sensors
  Radar radar_;
  Lidar lidar_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;
};

#endif /* FusionUKF_H_ */
