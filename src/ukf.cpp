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

  ///* Sigma point spreading parameter
  lambda_ = 2.4 - n_aug_;

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
  std_a_ = 0.50; // change this was 30

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.50; // change this was 30

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
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  ///* Weights of sigma points
//  weights_ = VectorXd(2*n_aug_+1);

  weights_ = VectorXd::Constant(2 * n_aug_ + 1, 1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "UKF: " << endl;
    double px = 0;
    double py = 0;
    double v = 0;
    double yaw = 0;
    double yawd = 0;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
        double ro = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];
        double ro_dot = meas_package.raw_measurements_[2];

        px = ro * cos(phi);
        py = ro * sin(phi);
        v = ro_dot;
        yaw = phi;
        yawd = 0;

        // High confidence in px, py, v and yaw
        P_ <<   1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        px = meas_package.raw_measurements_[0];
        py = meas_package.raw_measurements_[1];
        v = 0.0;
        yaw = (px == 0 && py == 0) ? 0 : atan2(py, px);
        yawd = 0;

        // High confidence in px, py and yaw
        P_ <<   0.1, 0, 0, 0, 0,
                0, 0.1, 0, 0, 0,
                0, 0, 0.01, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;
    }

    x_ << px, py, v, yaw, yawd;


    previous_timestamp_ = meas_package.timestamp_ ;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

//   - Time is measured in seconds.

  double dt = (meas_package.timestamp_ - previous_timestamp_)/1000000.0 ;

  previous_timestamp_ = meas_package.timestamp_ ;
  Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // Use the sensor type to perform the update step.
  // Update the state and covariance matrices.
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
      //create example vector for incoming radar measurement
  //    VectorXd z = VectorXd(n_z);
  //    z = Zsig.col(0);
          //rho in m
          //phi in rad
          //rho_dot in m/s
    std::cout << "Update Radar" << std::endl;
//    UpdateRadar(meas_package.raw_measurements_);


  } else {
    // Laser updates
    std::cout << "Update Lidar" << std::endl;
//    UpdateLidar(meas_package.raw_measurements_);
  }
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

  //1. Generate Sigma Points
//  MatrixXd Xsig_in = MatrixXd(n_x_, 2 * n_x_ + 1);
//  GenerateSigmaPoints(Xsig_in); // Generate Sigma Points

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
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

  /**
  TODO:

  Complete this function!
  Use lidar data to update the belief about the object's
  position.

  Modify the state vector, x_,

  and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

// Initialise variables
  int n_z = 2; // dimension of measurement state vector for radar

  // matrix with sigma points in measurement space
  MatrixXd Zsig_out = MatrixXd(n_z, 2 * n_aug_ + 1);    // matrix 3 x 15

  // mean predicted measurement
  VectorXd z_out = VectorXd(n_z);

  // measurement covariance matrix S
  MatrixXd S_out = MatrixXd(n_z,n_z);


  // Predict Radar measurement
  PredictLidarMeasurement(Zsig_out, z_out, S_out);

  // Update state and covariance matrix
  UpdateState(z, Zsig_out, z_out, S_out, n_z);


}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const VectorXd &z) {
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


  // Initialise variables
  int n_z = 3; // dimension of measurement state vector for radar

  // matrix with sigma points in measurement space
  MatrixXd Zsig_out = MatrixXd(n_z, 2 * n_aug_ + 1);    // matrix 3 x 15

  // mean predicted measurement
  VectorXd z_out = VectorXd(n_z);

  // measurement covariance matrix S
  MatrixXd S_out = MatrixXd(n_z,n_z);


  // Predict Radar measurement
  PredictRadarMeasurement(Zsig_out, z_out, S_out);

  // Update state and covariance matrix
  UpdateState(z, Zsig_out, z_out, S_out, n_z);
}


//void UKF::GenerateSigmaPoints(MatrixXd& Xsig_out) {

//  //create sigma point matrix
//  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

//  //calculate square root of P
//  MatrixXd A = P_.llt().matrixL();

//  // set lambda (spreading parameter)
//  lambda_ = 3 - n_x_;
//  //calculate sigma points ...
//  //set sigma points as columns of matrix Xsig

//  // first sigma point is x
//  Xsig.col(0) = x_;
//  double sqlpn =  sqrt(lambda_+n_x_);// sqrt(lambda+nx)

//  //set remaining sigma points
//  for (int i = 0; i < n_x_; i++)
//  {
//    Xsig.col(i+1)     = x_ + sqlpn * A.col(i);  // 2nd sigma point through nk+1
//    Xsig.col(i+1+n_x_) = x_ - sqlpn * A.col(i);  // nk+2 sigma point through 2nk+1
//  }

//  //write result
//  Xsig_out = Xsig;
//}

void UKF::AugmentedSigmaPoints(MatrixXd& Xsig_aug) {

  //define spreading parameter
//  double lambda = 3 - n_aug_;

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //create augmented mean state
  x_aug.head(n_x_) << x_; // don't need to set last two elemts because they are zero already
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.topLeftCorner(n_x_,n_x_) = P_;

//  MatrixXd Q = MatrixXd(2,2);

//   Q <<  pow(std_a_,2), 0,
//         0, pow(std_yawdd_,2);

//   P_aug.bottomRightCorner(2,2) << Q;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
  std::cout << P_aug << std::endl;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) << x_aug;

  double sqlpn =  sqrt(lambda_+n_aug_);// sqrt(lambda+nx)

  //set remaining sigma points
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)     = x_aug + sqlpn * A.col(i);  // 2nd sigma point through nk+1
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqlpn * A.col(i);  // nk+2 sigma point through 2nk+1
  }

  //print result
//  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
}

void UKF::SigmaPointPrediction(MatrixXd& Xsig_in, double delta_t) {

  VectorXd x_k = VectorXd(n_x_);
  VectorXd F = VectorXd(n_x_);
  VectorXd noise = VectorXd(n_x_);
  //predict sigma points
  for (int i=0; i< (2 * n_aug_ + 1); i++)
  {
      x_k = Xsig_in.col(i).head(n_x_);

      double v = x_k(2);
      double yaw = x_k(3);
      double yawd = x_k(4);
      double nu_a = Xsig_in(5,i);
      double nu_yawdd = Xsig_in(6,i);

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

  //print result
//  std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;
}

void UKF::PredictMeanAndCovariance(){

  // set lambda (spreading parameter)
//  lambda_ = 3 - n_aug_;

//    weights_(0) = lambda_ / (lambda_ + n_aug_);
    x_ = weights_(0)*Xsig_pred_.col(0);
    for (int i =1; i< 2*n_aug_+1; i++)
    {
      //set weights
//      weights_(i) = 0.5 / (lambda_ + n_aug_);

      //predict state mean
      x_ += weights_(i) * Xsig_pred_.col(i);
    }

  //predict state covariance matrix
    for (int i =0; i< 2*n_aug_+1; i++)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
//        std::cout << x_diff << std::endl;
        //angle normalization
//        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI; // it gets stuck in here
//        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        x_diff(3) = remainder (x_diff(3),M_PI);

        P_ += weights_(i) * x_diff * x_diff.transpose();
    }
}

void UKF::PredictLidarMeasurement(MatrixXd& Zsig_out, VectorXd& z_out, MatrixXd& S_out) {

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;


  //create matrix for sigma points in measurement space
  // no need to generate measurement sigma points Zsig as we can use directly the Xsig
  // we only have to add noise to the values to mimic the sensor output
  //  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // here we assume noise is zero; we account for the noise in the covariance matrix
  VectorXd noise_w = VectorXd(n_z);     // matrix 3 x 1


  MatrixXd z_meas = MatrixXd(n_z, 2 * n_aug_ + 1);    // matrix 3 x 15

  for (int i =0; i< 2 * n_aug_ + 1; i++)
  {
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      VectorXd h_x = VectorXd(n_z);

      h_x << px,
            py;     // matrix 2 x 1

      //transform sigma points into measurement space
      z_meas.col(i) = h_x + noise_w;

      //calculate mean predicted measurement
      z_pred += weights_(i) * z_meas.col(i);

  }

  for (int i =0; i< 2 * n_aug_ + 1; i++)
  {
      //calculate measurement covariance matrix S
      VectorXd z_diff = VectorXd(n_z);
      z_diff = z_meas.col(i) - z_pred;
      //angle normalization
//      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
//      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
      z_diff(1) = remainder (z_diff(1),M_PI);
      S += weights_(i) * z_diff * z_diff.transpose();
  }

  // compute R matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;

  S += R;

  //print result
//  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
//  std::cout << "S: " << std::endl << S << std::endl;

  //write result
  Zsig_out = z_meas; // sigma points in measurement space (we need this for the update step)
  z_out = z_pred; // mean predicted value
  S_out = S; // covariance matrix of predicted measurement
}


void UKF::PredictRadarMeasurement(MatrixXd& Zsig_out, VectorXd& z_out, MatrixXd& S_out) {

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // set lambda (spreading parameter)
//  lambda_ = 3 - n_aug_;

  //create matrix for sigma points in measurement space
  // no need to generate measurement sigma points Zsig as we can use directly the Xsig
  // we only have to add noise to the values to mimic the sensor output
  //  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // here we assume noise is zero; we account for the noise in the covariance matrix
  VectorXd noise_w = VectorXd(n_z);     // matrix 3 x 1


  MatrixXd z_meas = MatrixXd(n_z, 2 * n_aug_ + 1);    // matrix 3 x 15

  for (int i =0; i< 2 * n_aug_ + 1; i++)
  {
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);

      double ro = sqrt(px*px + py*py);
      double phi = (px == 0 && py == 0) ? 0 :atan2(py,px);
      double ro_dot = (ro == 0) ? 0 :(px * cos(yaw) * v + py * sin(yaw) * v ) / ro;

      VectorXd h_x = VectorXd(n_z);  // matrix 3 x 5

      h_x << ro,
            phi,
            ro_dot;      // matrix 3 x 1

      //transform sigma points into measurement space
      z_meas.col(i) = h_x + noise_w;

      //calculate mean predicted measurement
      z_pred += weights_(i) * z_meas.col(i);

  }

  for (int i =0; i< 2 * n_aug_ + 1; i++)
  {
      //calculate measurement covariance matrix S
      VectorXd z_diff = VectorXd(n_z);
      z_diff = z_meas.col(i) - z_pred;
      //angle normalization
//      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
//      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
      z_diff(1) = remainder (z_diff(1),M_PI);
      S += weights_(i) * z_diff * z_diff.transpose();
  }

  // compute R matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;

  S += R;

  //print result
//  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
//  std::cout << "S: " << std::endl << S << std::endl;

  //write result
  Zsig_out = z_meas; // sigma points in measurement space (we need this for the update step)
  z_out = z_pred; // mean predicted value
  S_out = S; // covariance matrix of predicted measurement
}


void UKF::UpdateState(const VectorXd& z, MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S, int n_z){

  // RADAR ONLY

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  //calculate cross correlation matrix
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
      VectorXd z_diff = VectorXd(n_z);
      z_diff = Zsig.col(i) - z_pred;
      //angle normalization
//      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
//      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
      z_diff(1) = remainder (z_diff(1),M_PI);
      VectorXd x_diff = VectorXd(n_z);
      x_diff = Xsig_pred_.col(i) - x_;

      //angle normalization
//      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
//      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
      x_diff(3) = remainder (x_diff(3),M_PI);
      Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
//  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
//  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  z_diff(1) = remainder (z_diff(1),M_PI);
  //update state mean and covariance matrix
  x_ << x_ + K * z_diff;
  P_ << P_ - K * S * K.transpose();

  //print result
//  std::cout << "Updated state x: " << std::endl << x_ << std::endl;
//  std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;

}
