#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#define EPS 0.001; // A small number

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.57;

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

  // State dimension
  n_x_ = x_.size();

  // Augmented s dimension
  n_aug_ = n_x_ + 2;

  // Sigma points
  n_sig_ = 2 * n_aug_ + 1;

  // Predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // Spreading parameter lambda
  lambda_ = 3 - n_aug_;

  // Sigma point weights
  weights_ = VectorXd(n_sig_);

  // Measurement noise covariance matices
  // Radar matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << pow(std_radr_, 2), 0,                   0,
              0,                 pow(std_radphi_, 2), 0,
              0,                 0,                   pow(std_radrd_, 2);

  // Radar matrix
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << pow(std_laspx_, 2), 0,
              0,                  pow(std_laspy_, 2);
}

UKF::~UKF() {}

/**
 * Normalizes angle to [-pi, pi]
 */
void UKF::NormalizeAng(double *ang) {
  while (*ang > M_PI)  *ang -= 2.0 * M_PI;
  while (*ang < -M_PI) *ang += 2.0 * M_PI;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  // Initialize
  if (!is_initialized_) {
    // Covariance matrix
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    // Radar measurement
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Conver from polar to cartesian coordinates
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float rho_dot = meas_package.raw_measurements_[3];
      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      float v = sqrt(pow(vx, 2) + pow(vy, 2));
      // Initialize state
      x_ << px, py, v, 0, 0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // Use 0 for the first measurement of velocity as we don't know it
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];
      // Initialize state
      x_ << px, py, 0, 0, 0;
      // Make sure we don't divide by 0
      if (fabs(x_(0)) < EPS && fabs(x_(1)) < EPS) {
        x_(0) = EPS;
        x_(1) = EPS;
      }
    }

    // Initialize the weights
    // First weight
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    // Subsequent weights
    for (int i = 1; i < weights_.size(); i++) {
      weights_(i) = 0.5 / (lambda_ + n_aug_);
    }

    // Calculate dt: save initial timestamp
    time_us_ = meas_package.timestamp_;

    // Initialization complete
    is_initialized_ = true;

    return;
  }

  // Calculate dt
  double dt = (meas_package.timestamp_ - time_us_);
  dt /= 1000000.0; // convert micros to s
  time_us_ = meas_package.timestamp_;

  // Prediction step
  Prediction(dt);

  // Update step
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
