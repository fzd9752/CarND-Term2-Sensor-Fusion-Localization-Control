#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

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
  P_ = MatrixXd::Identity(5, 5);

  P_(2,2) = 12;
  P_(3,3) = 2;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.4;

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

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  n_x_ = 5;

  n_aug_ = 7;

  lambda_ = 3 - n_aug_;

  ///* weights_ of sigma points
  weights_ = VectorXd(2 * n_aug_ +1);
  weights_(0) = lambda_/(lambda_ + n_aug_);

  for (int i = 1; i < 2 * n_aug_ +1; i++)
  {
    weights_(i) = 0.5/(lambda_ + n_aug_);
  }

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  time_us_ = 0.0;
  is_initialized_ = false;

  // set measurements noise
  R_radar_ = MatrixXd(3, 3);
  R_radar_.fill(0.0);
  R_radar_(0,0) = std_radr_*std_radr_;
  R_radar_(1,1) = std_radphi_*std_radphi_;
  R_radar_(2,2) = std_radrd_*std_radrd_;

  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  MatrixXd H_ = MatrixXd(2,5);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;


}

UKF::~UKF() {}

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
  if (!is_initialized_) {

    x_ <<  1.0,
           1.0,
           0.0,
           0.0,
           0.0;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
 	    //Convert radar from polar to cartesian coordinates and initialize state.
      const float ro = meas_package.raw_measurements_[0];
 	    const float phi = meas_package.raw_measurements_[1];
      // Initialize radar state.
 	    float px = ro * cos(phi);
      float py = ro * sin(phi);

      if(fabs(px)<0.0001){
        px = 0.0001;
        P_(0,0) = 1000;
      }

      if(fabs(py)<0.0001){
        py = 0.0001;
        P_(1,1) = 1000;
      }

      x_(0) = px;
      x_(1) = py;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
 	    //Initialize lidar state.

      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];

      if(fabs(px)<0.0001){
        px = 0.0001;
        P_(0,0) = 1000;
      }

      if(fabs(py)<0.0001){
        py = 0.0001;
        P_(1,1) = 1000;
      }

      x_(0) = px;
      x_(1) = py;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;
  }

  //dt - expressed in seconds
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    // Radar updates
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    // Laser updates
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

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(&Xsig_aug);

  SigmaPointPrediction(Xsig_aug, &Xsig_pred_ , delta_t);

  PredictMeanAndCovariance(&x_, &P_);


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
  std::cout << "using lidar..." << '\n';

  int n_z = 2;

  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

  //std::cout << "z: " << z << '\n';

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);

  for (int i=0; i < 2*n_aug_+1; i++ ) {
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);

  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  for (int i=0; i < 2*n_aug_+1; i++ )  {
    VectorXd z_diff0 = Zsig.col(i)-z_pred;

    S = S + weights_(i) * z_diff0 * z_diff0.transpose();

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    NormaliseAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff0.transpose();
  }

  S = S + R_lidar_;

  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;

  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  float NIS_lidar_ = z_diff.transpose()*S.inverse()*z_diff;

  std::cout << "Lidar Update x" << '\n';
  std::cout << x_ << '\n';
  std::cout << "Lidar Update P" << '\n';
  std::cout << P_ << '\n';
  std::cout << "NIS_lidar: " << NIS_lidar_ << endl;

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

  std::cout << "using radar update..." << '\n';

  int n_z = 3;

  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0],
       meas_package.raw_measurements_[1],
       meas_package.raw_measurements_[2];

  //std::cout << "z: " << z << '\n';

  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++ )
  {
    //set state
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    //transform sigma points into measurement space
    Zsig(0, i) = sqrt(px*px + py*py);
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px * cos(yaw) * v + py * sin(yaw) *v)/sqrt(px*px + py*py);
    //calculate mean predicted measurement
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  //std::cout << "Zsig: " << endl << Zsig << '\n';
  //std::cout << "z_pred: "  << z_pred << '\n';

  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  for (int i=0; i < 2*n_aug_+1; i++ )
  {
    VectorXd z_diff0 = Zsig.col(i)-z_pred;
    NormaliseAngle(z_diff0(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    NormaliseAngle(x_diff(3));

    S = S + weights_(i) * z_diff0 * z_diff0.transpose();

    Tc = Tc + weights_(i) * x_diff * z_diff0.transpose();

  }
  //std::cout << "S" << endl << S << '\n';
  //std::cout << "Tc" << endl << Tc << '\n';

  //std::cout << "R_radar_: " << R_radar_ << '\n';

  S = S + R_radar_;
  //std::cout << "S+meas erro" << endl << S << '\n';


  MatrixXd K = Tc * S.inverse();
  //std::cout << "K" << endl << K << '\n';

  VectorXd z_diff = z - z_pred;
  NormaliseAngle(z_diff(1));

  //std::cout << "z_diff " << z_diff << '\n';

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  float NIS_radar_ = z_diff.transpose()*S.inverse()*z_diff;

  std::cout << "Radar Update x" << '\n';
  std::cout << x_ << '\n';
  std::cout << "Radar Update P" << '\n';
  std::cout << P_ << '\n';
  std::cout << "NIS_radar: " << NIS_radar_ << endl;
}

void UKF::EKFUpdateLidar(MeasurementPackage meas_package)
{
  VectorXd z = VectorXd(2);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_lidar_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K * H_) * P_;

  std::cout << "Lidar EKF Update x" << '\n';
  std::cout << x_ << '\n';
  std::cout << "Lidar EKF Update P" << '\n';
  std::cout << P_ << '\n';
}
/**
 * Helpful function
 */
void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out)
{
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  MatrixXd L = P_aug.llt().matrixL();

  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd Xsig_aug, MatrixXd* Xsig_out, double delta_t)
{
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  double t2 = delta_t*delta_t;

  for (int i=0; i < 2 * n_aug_ + 1; i++)
  {
      double px = Xsig_aug(0, i);
      double py = Xsig_aug(1, i);
      double v = Xsig_aug(2, i);
      double yaw = Xsig_aug(3, i);
      double yawd = Xsig_aug(4, i);
      double nu_a = Xsig_aug(5, i);
      double nu_yawdd = Xsig_aug(6, i);

      double px_p, py_p;

      if (fabs(yawd) > 0.001) {
          px_p = px + v/yawd*(sin(yaw+yawd*delta_t)-sin(yaw));
          py_p = py + v/yawd*(cos(yaw)-cos(yaw+yawd*delta_t));
      } else {
          px_p = px + v*cos(yaw)*delta_t;
          py_p = py +v*sin(yaw)*delta_t;
      }

      double v_p = v;
      double yaw_p = yaw + yawd*delta_t;
      double yawd_p = yawd;

      px_p = px_p + 0.5*t2*cos(yaw)*nu_a;
      py_p = py_p + 0.5*t2*sin(yaw)*nu_a;
      v_p = v_p + delta_t*nu_a;
      yaw_p = yaw_p +0.5*t2*nu_yawdd;
      yawd_p = yawd_p + delta_t*nu_yawdd;

      Xsig_pred(0, i) = px_p;
      Xsig_pred(1, i) = py_p;
      Xsig_pred(2, i) = v_p;
      Xsig_pred(3, i) = yaw_p;
      Xsig_pred(4, i) = yawd_p;
   }

  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;
  *Xsig_out = Xsig_pred;
}

void UKF::PredictMeanAndCovariance(VectorXd* x_pred, MatrixXd* P_pred)
{
  VectorXd x = VectorXd(n_x_);
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x+ weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    NormaliseAngle(x_diff(3));

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;

  *x_pred = x;
  *P_pred = P;
}

void UKF::NormaliseAngle(double& phi)
{
  //std::cout << "phi_0: " << phi << '\n';
  phi = atan2(sin(phi), cos(phi));
  //std::cout << "phi: " << phi << '\n';

  //angle normalization
  //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
  //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
}
