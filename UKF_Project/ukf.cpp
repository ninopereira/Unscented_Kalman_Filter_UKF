#include "ukf.h"
#include "tools.h"
#include "../Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//  UKF Roadmap
//
//    A. Prediction
//        1. Generate Sigma Points  (see Generating_Sigma_Points Project)
//        2. Predict Sigma Points (see Sigma_Point_Prediction Project)
//        3. Predict Mean and Covariance (see Predicted_Mean_Cov Project)

//    B. Update
//        1. Predict Measurement (see Predict_Radar_Meas Project)
//        2. Update State (see UKF_Update)

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  ///* State dimension
  int n_x_ = 5;

  ///* Augmented state dimension
  int n_aug_ = 7;

  ///* Sigma point spreading parameter
  double lambda_ = 3 - n_x_;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  ///* time when the state is true, in us
  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.30; // change this was 30

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.30; // change this was 30

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  ///* Weights of sigma points
  weights_ = VectorXd();

  ///* the current NIS for radar
  double NIS_radar_ = 0;

  ///* the current NIS for laser
  double NIS_laser_ = 0;


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function!

  Make sure you switch between lidar and radar
  measurements.
  */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

//  UKF Roadmap
//
//    A. Prediction
//        1. Generate Sigma Points  (see Generating_Sigma_Points Project)
//        2. Predict Sigma Points (see Sigma_Point_Prediction Project)
//        3. Predict Mean and Covariance (see Predicted_Mean_Cov Project)

  //1. Generate Sigma Points
  MatrixXd Xsig_in = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  GenerateSigmaPoints(&Xsig_in); // Generate Sigma Points

  // 2. Predict Sigma Points
  SigmaPointPrediction(&Xsig_pred_, &Xsig_in, delta_t);

  // 3. Predict Mean and Covariance
  PredictMeanAndCovariance(&x_, &P_);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
//    B. Update
//        1. Predict Measurement (see Predict_Radar_Meas Project)
//        2. Update State (see UKF_Update)

  /**
  TODO:

  Complete this function!
  Use lidar data to update the belief about the object's
  position.

  Modify the state vector, x_,

  and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
//    B. Update
//        1. Predict Measurement (see Predict_Radar_Meas Project)
//        2. Update State (see UKF_Update)

  /**
  TODO:

  Complete this function!
  Use radar data to update the belief about the object's
  position.

  Modify the state vector, x_,

  and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}


void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //calculate sigma points ...
  //set sigma points as columns of matrix Xsig

  // first sigma point is x
  Xsig.col(0) << x_;
  double sqlpn =  sqrt(lambda_+n_x_);// sqrt(lambda+nx)

  //set first column of sigma point matrix
  Xsig.col(0)  = x_;

  //set remaining sigma points
  for (int i = 0; i < n_x_; i++)
  {
    Xsig.col(i+1)     = x_ + sqlpn * A.col(i);  // 2nd sigma point through nk+1
    Xsig.col(i+1+n_x_) = x_ - sqlpn * A.col(i);  // nk+2 sigma point through 2nk+1
  }

  //write result
  *Xsig_out = Xsig;
}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out, MatrixXd* Xsig_in, double delta_t) {

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  VectorXd x_k = VectorXd(n_x_);
  VectorXd F = VectorXd(n_x_);
  VectorXd noise = VectorXd(n_x_);
  //predict sigma points
  for (int i=0; i< (2 * n_aug_ + 1); i++)
  {
      x_k = Xsig_in->col(i).head(n_x_);

      double v = x_k(2);
      double yaw = x_k(3);
      double yawd = x_k(4);
      double nu_a = (*Xsig_in)(5,i);
      double nu_yawdd = (*Xsig_in)(6,i);

      if (fabs(yawd) < 0.001)
      {
          F <<  v * delta_t * cos(yaw),
                v * delta_t * sin(yaw),
                0,
                yawd * delta_t,
                0;
      }
      else //avoid division by zero
      {
          F <<  v/yawd * (sin(yaw + yawd * delta_t) - sin(yaw)),
                v/yawd * (cos(yaw) - cos(yaw + yawd*delta_t)),
                0,
                yawd*delta_t,
                0;
      }

      noise <<  0.5 * nu_a * delta_t*delta_t * cos(yaw),
                0.5 * nu_a * delta_t*delta_t * sin(yaw),
                nu_a * delta_t,
                0.5 * nu_yawdd * delta_t*delta_t,
                nu_yawdd * delta_t;


      //write predicted sigma points into right column
      Xsig_pred.col(i) = x_k + F + noise;
  }

  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  //write result
  *Xsig_out = Xsig_pred;

}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

    weights_(0) = lambda_ / (lambda_ + n_aug_);
    x = weights_(0)*Xsig_pred_.col(0);
    for (int i =1; i< 2*n_aug_+1; i++)
    {
      //set weights
      weights_(i) = 0.5 / (lambda_ + n_aug_);

      //predict state mean
      x += weights_(i) * Xsig_pred_.col(i);
    }

  //predict state covariance matrix
    for (int i =0; i< 2*n_aug_+1; i++)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        P += weights_(i) * x_diff * x_diff.transpose();
    }

  //print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;
}
