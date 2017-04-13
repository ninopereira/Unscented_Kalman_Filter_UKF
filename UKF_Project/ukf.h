#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "../Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;
  long long  previous_timestamp_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(const VectorXd &z);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param z The RADAR measurement raw data at k+1
   */
  void UpdateRadar(const VectorXd &z);

   /**
   * Predicts the sigma points by passing generated sigma points throught the non-linear process function
   * @param Xsig_out output matrix containing all predicted sigma points
   * @param Xsig_in input matrix containing all generated sigma points
   * @param delta_t elapsed time between the current and previous iteration
   */
  void SigmaPointPrediction(MatrixXd& Xsig_in, double delta_t);

   /**
   * Generates the sigma points
   * @param Xsig_out output matrix containing generated sigma points
   */
  void GenerateSigmaPoints(MatrixXd& Xsig_out);

   /**
   * Predicts mean and covariance of the state
   * @param x_out output mean state
   * @param P_out output covariance matrix
   */
  void PredictMeanAndCovariance();

  /**
   * Computes the predicted measurement for RADAR measurement type
   * @param Zsig_out (output) predicted sigma points matrix
   * @param z_out (output) mean value of predicted measurement
   * @param S (output) measurement covariance matrix
   */
  void PredictRadarMeasurement(MatrixXd& Zsig_out, VectorXd& z_out, MatrixXd& S_out);

   /**
   * Updates the state (x_) and covariance matrix (P_)
   * @param z The RADAR measurement raw data at k+1
   * @param Zsig (input) predicted sigma points matrix
   * @param z_pred (input) mean value of predicted measurement
   * @param S (input) measurement covariance matrix
   * @param n_z (input) measurement state dimension
   */
  void UpdateState(const VectorXd& z, MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S, int n_z);

};

#endif /* UKF_H */
