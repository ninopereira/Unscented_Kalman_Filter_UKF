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
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  // number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd::Zero(n_x_);

  // initial covariance matrix
  P_ = MatrixXd::Zero(n_x_, n_x_);

  ///* time when the state is true, in us
  previous_timestamp_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2; // change this was 30

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.7; // change this was 30

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
  Xsig_pred_ = MatrixXd::Zero(n_x_, n_sig_);

  ///* Weights of sigma points
  weights_ = VectorXd::Zero(n_sig_);

  weights_(0) = double(lambda_ / (lambda_ + n_aug_));
  for (int i=1; i < n_sig_; i++)
  {
    weights_(i) = double(0.5 / (lambda_ + n_aug_));
  }
  Q_ = MatrixXd(2,2);

  Q_ <<  std_a_ * std_a_, 0,
         0, std_yawdd_ * std_yawdd_;

  // Radar R matrix
  R_radar_ = MatrixXd(3,3);
  R_radar_ <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;

  // Laser R matrix
  R_laser_ = MatrixXd(2,2);
  R_laser_ <<  std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;

  ///* the current NIS for radar
  NIS_radar_ = 0;

  ///* the current NIS for laser
  NIS_laser_ = 0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The

 latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {

    // first measurement
    cout << "UKF: " << endl;
    double px = 0;
    double py = 0;
    double v = 0;
    double yaw = 0;
    double yawd = 0;

    // High confidence in px, py, v and yaw
    P_ <<   1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
        double ro = meas_package.raw_measurements_(0);
        double phi = meas_package.raw_measurements_(1);
        double ro_dot = meas_package.raw_measurements_(2);

        px = ro * cos(phi);
        py = ro * sin(phi);
        v = ro_dot;
        yaw = 0.0;
        yawd = 0.0;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        px = meas_package.raw_measurements_(0);
        py = meas_package.raw_measurements_(1);
        v = 0.0;
        yaw = 0.0;
        yawd = 0.0;
    }

    x_ << px, py, v, yaw , yawd;

    previous_timestamp_ = meas_package.timestamp_ ;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

//   - Time is measured in seconds.
  double delta_t = ((double)(meas_package.timestamp_ - previous_timestamp_))/1000000.0 ;

// Deal with large periods of missing data from sensors
  while (delta_t > 0.5)
  {
    Prediction(0.5);
    delta_t = delta_t-0.5;
  }

  Prediction(delta_t);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // Use the sensor type to perform the update step.
  // Update the state and covariance matrices.
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package.raw_measurements_);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
    UpdateLidar(meas_package.raw_measurements_);
  }
    previous_timestamp_ = meas_package.timestamp_ ;
}


///**
// * Predicts sigma points, the state, and the state covariance matrix.
// * @param {double} delta_t the change in time (in seconds) between the last
// * measurement and this one.
// */
void UKF::Prediction(double delta_t) {

//  UKF Roadmap
//
//    A. Prediction
//        1. Generate Sigma Points  (see Generating_Sigma_Points Project)
//        2. Predict Sigma Points (see Sigma_Point_Prediction Project)
//        3. Predict Mean and Covariance (see Predicted_Mean_Cov Project)

  //1. Generate Augmented Sigma Points
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, n_sig_);

  AugmentedSigmaPoints(Xsig_aug);

  // 2. Predict Sigma Points
  SigmaPointPrediction(Xsig_aug, delta_t);

  // 3. Predict Mean and Covariance
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const VectorXd &z) {
//    B. Update
//        1. Predict Measurement (see Predict_Radar_Meas Project)
//        2. Update State (see UKF_Update)

// Initialise variables
  int n_z = 2; // dimension of measurement state vector for radar

  MatrixXd z_meas = MatrixXd::Zero(n_z, n_sig_);    // matrix 2 x 15

  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // noise matrix
  VectorXd noise_w = VectorXd::Zero(n_z);     // matrix 2 x 1

// calculate z_pred
  for (int i =0; i< n_sig_; i++)
  {
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      VectorXd h_x = VectorXd(n_z);

      h_x << px,
            py;     // matrix 2 x 1

      //transform sigma points into measurement space
      z_meas.col(i) = h_x + noise_w;

      //calculate mean predicted measurement
      z_pred = z_pred + weights_(i) * z_meas.col(i);
  }


// calculate measurement covariance matrix S and cross-correlation matrix Tc
  for (int i =0; i< n_sig_; i++)
  {
      // Calculate measurement covariance matrix S
      VectorXd z_diff = VectorXd(n_z);
      z_diff = z_meas.col(i) - z_pred;
      S = S + weights_(i) * z_diff * z_diff.transpose();

      // Update Tc
      VectorXd x_diff = VectorXd(n_z);
      x_diff = Xsig_pred_.col(i) - x_;

      Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  S = S + R_laser_;

  // UPDATE STATE

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // calculate the lidar NIS.
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const VectorXd &z) {
//    B. Update
//        1. Predict Measurement (see Predict_Radar_Meas Project)
//        2. Update State (see UKF_Update)

  // Initialise variables
  int n_z = 3; // dimension of measurement state vector for radar

  // matrix with sigma points in measurement space
  MatrixXd z_meas = MatrixXd::Zero(n_z, n_sig_);    // matrix 3 x 15

  // mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // here we assume noise is zero; we account for the noise in the covariance matrix
  VectorXd noise_w = VectorXd::Zero(n_z);     // matrix 3 x 1

  // Predict Radar measurement
  for (int i =0; i< n_sig_; i++)
  {
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);

      px = (fabs(px)<0.001) ? 0.001 : px;
      py = (fabs(py)<0.001) ? 0.001 : py;

      double ro = sqrt(px*px + py*py);
      ro = (fabs(ro)<0.001) ? 0.001 : ro;
      double phi = atan2(py,px);
      double ro_dot = (px * cos(yaw) * v + py * sin(yaw) * v ) / ro;

      VectorXd h_x = VectorXd(n_z);  // matrix 3 x 5

      h_x << ro,
            phi,
            ro_dot;      // matrix 3 x 1

      //transform sigma points into measurement space
      z_meas.col(i) = h_x + noise_w;

      //calculate mean predicted measurement
      z_pred = z_pred + weights_(i) * z_meas.col(i);
  }

  for (int i =0; i< n_sig_; i++)
  {
      //calculate measurement covariance matrix S
      VectorXd z_diff = VectorXd(n_z);
      z_diff = z_meas.col(i) - z_pred;

      //angle normalization
      z_diff(1) = remainder (z_diff(1),M_PI); // RADAR

      S = S + weights_(i) * z_diff * z_diff.transpose();

      VectorXd x_diff = VectorXd(n_z);
      x_diff = Xsig_pred_.col(i) - x_;

      //angle normalization
      x_diff(3) = remainder (x_diff(3),M_PI); // RADAR

      //compute cross-correlation matrix
      Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  S = S + R_radar_;

  // UPDATE STATE

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = remainder (z_diff(1),M_PI); // RADAR
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // calculate the radar NIS.
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}

void UKF::AugmentedSigmaPoints(MatrixXd& Xsig_aug) {
  //create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);

  //create augmented mean state
  x_aug.head(n_x_) = x_; // don't need to set last two elemts because they are zero already
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug.bottomRightCorner(2,2) = Q_;

  //calculate square root of P
  MatrixXd A = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;

  double sqlpn =  sqrt(lambda_+n_aug_);// sqrt(lambda+nx)

  //set remaining sigma points
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)     = x_aug + sqlpn * A.col(i);  // 2nd sigma point through nk+1
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqlpn * A.col(i);  // nk+2 sigma point through 2nk+1
  }
}


void UKF::SigmaPointPrediction(MatrixXd& Xsig_aug, double delta_t) {

  VectorXd x_k = VectorXd::Zero(n_x_);
  VectorXd F = VectorXd::Zero(n_x_);
  VectorXd noise = VectorXd::Zero(n_x_);

  //predict sigma points
  for (int i=0; i< (n_sig_); i++)
  {
      x_k = (Xsig_aug.col(i)).head(n_x_);

      double v = x_k(2);
      double yaw = x_k(3);
      double yawd = x_k(4);
      double nu_a = Xsig_aug(5,i);
      double nu_yawdd = Xsig_aug(6,i);

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
                v/yawd * (cos(yaw) - cos(yaw + yawd * delta_t)),
                0,
                yawd * delta_t,
                0;
      }

      noise <<  0.5 * nu_a * delta_t * delta_t * cos(yaw),
                0.5 * nu_a * delta_t * delta_t * sin(yaw),
                nu_a * delta_t,
                0.5 * nu_yawdd * delta_t * delta_t,
                nu_yawdd * delta_t;

      //write predicted sigma points into right column
      Xsig_pred_.col(i) = x_k + F + noise;
  }

}

void UKF::PredictMeanAndCovariance(){

    // reset the state and covariance matrices
    x_.fill(0.0);
    P_.fill(0.0);

// the following block can be replaced by the line bellow
//    for (int i =0; i< n_sig_; i++)
//    {
//      //predict state mean
//      x_ = x_ + weights_(i) * Xsig_pred_.col(i);
//    }
    x_ = Xsig_pred_ * weights_;

  //predict state covariance matrix
    for (int i =0; i< n_sig_; i++)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        x_diff(3) = remainder (x_diff(3),M_PI);
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}
