#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#define EPS 0.001 // A small number

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

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
  R_radar_ << std_radr_*std_radr_, 0,                       0,
              0,                   std_radphi_*std_radphi_, 0,
              0,                   0,                       std_radrd_*std_radrd_;

  // Lidas matrix
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_, 0,
              0,                     std_laspy_*std_laspy_;
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
      float rho     = meas_package.raw_measurements_[0];
      float phi     = meas_package.raw_measurements_[1];
      float rho_dot = meas_package.raw_measurements_[3];

      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      float v  = sqrt(vx*vx + vy*vy);
      // Initialize state
      x_ << px, py, v, 0, 0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // Use 0 for the first measurement of velocity as we don't know it
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];
      // Initialize state
      x_ << px, py, 0, 0, 0;
      // Make sure we don't divide by 0
      if ((fabs(x_(0)) < EPS) && (fabs(x_(1)) < EPS)) {
        x_(0) = EPS;
        x_(1) = EPS;
      }
    }

    // Initialize the weights
    // First weight
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    // Subsequent weights
    for (int i = 1; i < weights_.size(); i++) {
      weights_(i) = 0.5 / (n_aug_ + lambda_);
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
  cout << "x_" << x_ << endl;
  cout << "NIS_laser_ " << NIS_laser_ << endl;
  cout << "NIS_radar_ " << NIS_radar_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  double delta_t_2 = delta_t*delta_t;
  // Aug mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  // Aug state covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  // Sigma points matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // Square root of P matrix
  MatrixXd L = P_aug.llt().matrixL();

  Xsig_aug.col(0) = x_aug;
  double sqrt_lambda_n_aug = sqrt(lambda_ + n_aug_);
  VectorXd sqrt_lambda_n_aug_L;
  for (int i = 0; i < n_aug_; i++) {
    sqrt_lambda_n_aug_L = sqrt_lambda_n_aug * L.col(i);

    Xsig_aug.col(i + 1)          = x_aug + sqrt_lambda_n_aug_L;
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt_lambda_n_aug_L;
  }

  // Predict the sigma points
  for (int i = 0; i < n_sig_; i++) {
    // Read values
    double p_x      = Xsig_aug(0,i);
    double p_y      = Xsig_aug(1,i);
    double v        = Xsig_aug(2,i);
    double yaw      = Xsig_aug(3,i);
    double yawd     = Xsig_aug(4,i);
    double nu_a     = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // Precalculate sin and cos for yaw
    double sin_yaw = sin(yaw);
    double cos_yaw = cos(yaw);
    double arg     = yaw + yawd * delta_t;

    // Predicted state values
    double px_p, py_p;

    if (fabs(yawd) > EPS) { // Don't divide by 0
      double v_yawd = v / yawd;
      px_p = p_x + v_yawd * (sin(arg) - sin_yaw);
      py_p = p_y + v_yawd * (cos_yaw - cos(arg));
    } else {
      double v_delta_t = v * delta_t;
      px_p = p_x + v_delta_t * cos_yaw;
      py_p = p_y + v_delta_t * sin_yaw;
    }

    double v_p = v;
    double yaw_p = arg;
    double yawd_p = yawd;

    // Add noise
    px_p   += 0.5 * nu_a * delta_t_2 * cos_yaw;
    py_p   += 0.5 * nu_a * delta_t_2 * sin_yaw;
    v_p    += nu_a * delta_t;
    yaw_p  += 0.5 * nu_yawdd * delta_t_2;
    yawd_p += nu_yawdd * delta_t;

    // Write predicted points
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  // Predicted state mean
  x_ = Xsig_pred_ * weights_;

  // Predicted state covariance matrix
  P_.fill(0.0);
  // for each sigma point
  for (int i = 0; i < n_sig_; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Normalize angle
    NormalizeAng(&(x_diff(3)));
    // Update P
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // Number of measured dimensions
  int n_z = 2;
  // Create matrix for sigma points and transform points into measurement space
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);
  UpdateUKF(meas_package, Zsig, n_z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // Number of measured dimensions
  int n_z = 3;
  // Create matrix for sigma points in the measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);
  // Transform points into the measurement space
  for (int i = 0; i < n_sig_; i++) {
    // Read values
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double v_1 = cos(yaw) * v;
    double v_2 = sin(yaw) * v;
    // Measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y); // rho
    Zsig(1,i) = atan2(p_y, p_x);                 // phi
    Zsig(2,i) = (p_x*v_1 + p_y*v_2) / Zsig(0,i); // rho_dot
  }

  UpdateUKF(meas_package, Zsig, n_z);
}

/**
 * General update function, common to both sensors
 * @param {MeasurementPackage} meas_package, {MatrixXd} Zsig, {int} n_z
 */
void UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z) {
  VectorXd z_pred = VectorXd(n_z);
  z_pred = Zsig * weights_;

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    // Residual vector
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // Normalize angle
    NormalizeAng(&(z_diff(1)));
    // Update S
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Noise covariance matrix R
  MatrixXd R = MatrixXd(n_z, n_z);
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    R = R_radar_;
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    R = R_lidar_;
  }

  // Add noise
  S = S + R;

  // Cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    // Residual vector
    VectorXd z_diff = Zsig.col(i) - z_pred;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Normalize angle
      NormalizeAng(&(z_diff(1)));
    }
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Normalize angle
    NormalizeAng(&(x_diff(3)));
    // Update Tc
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Raw measurements
  VectorXd z = meas_package.raw_measurements_;

  // Kalman gain matrix K
  MatrixXd K = Tc * S.inverse();

  // Residual
  VectorXd z_diff = z - z_pred;

  // Normalize angle if Radar
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    NormalizeAng(&(z_diff(1)));
  }

  // Update state mean matrix
  x_ = x_ + K * z_diff;
  // Update covariance matrix
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    NIS_radar_ = z.transpose() * S.inverse() * z_diff;
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    NIS_laser_ = z.transpose() * S.inverse() * z_diff;
  }
}
