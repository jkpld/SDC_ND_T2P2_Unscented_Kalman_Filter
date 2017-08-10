#ifndef Sensor_H_
#define Sensor_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/********************************/
/*   Abstact Sensor class       */
class Sensor {
public:

  // measurment noise covariance
  MatrixXd R;

  // Constructor
  Sensor() {};

  // Destructor
  virtual ~Sensor() {};

  /* Methods to be implimented by subclasses:
    Compute difference between measurment and predicted measurement from state
    Convert from state to measurment
    Convert from measurment to state
  */
  virtual MatrixXd state_to_measure(const MatrixXd &SigmaPoints) = 0;
  virtual VectorXd measure_to_state(const VectorXd &meas) = 0;

  VectorXd angleParams_;
};

/********************/
/*   Radar Sensor   */
class Radar : public Sensor
{
public:
  // Constructor
  Radar();
  // Destructor
  ~Radar();

  MatrixXd state_to_measure(const MatrixXd &SigmaPoints);
  VectorXd measure_to_state(const VectorXd &meas);
};

/********************/
/*   Lidar Sensor   */
class Lidar : public Sensor
{
public:
  // Constructor
  Lidar();
  // Destructor
  ~Lidar();

  MatrixXd state_to_measure(const MatrixXd &SigmaPoints);
  VectorXd measure_to_state(const VectorXd &meas);
};


#endif /* Sensor_H_ */
