#define _USE_MATH_DEFINES
#include "FusionEKF.h"
#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "tools.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;
/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  /*
   * TODO: Finish initializing the FusionEKF.
   */
  // the initial transition matrix F_
  kf_.F_ = MatrixXd(4, 4);
  kf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  /*
   * TODO: Set the process and measurement noises
   */
  // set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
}
/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     **/
    /*
     * You'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      
    /* From notes--this does the linearization maybe? */
   double p_x = measurement_pack.raw_measurements_[0];
   double p_y = measurement_pack.raw_measurements_[1];
 double v_x = measurement_pack.raw_measurements_[2];
 double v_y = measurement_pack.raw_measurements_[3];
   double rho = pow(pow(p_x, 2)+pow(p_y,2),-0.5);
   double phi = arctan(p_y/p_x);
 while(phi < -M_PI) {
     phi = phi + 2 * M_PI;
 }
 while(phi > M_PI)  {
     phi = phi - 2 * M_PI;
 }
      
   // rho_dot = (p_x * v_x + p_y * v_y) / rho;
   double rho_dot = 0;  
      
 ekf_.x_ << rho, phi, rho_dot;  // don't I need 4 not 3?
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;
    }
    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }
  /**
   * Prediction
   */
  
  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  kf_.F_(0, 2) = dt;
  kf_.F_(1, 3) = dt;
  
  // set the process covariance matrix Q
  kf_.Q_ = MatrixXd(4, 4);
  kf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
  ekf_.Predict();
    
  /**
   * Update
   */
  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.Update(measurement_pack.raw_measurements_);    
  } else {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);    
    
  }
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}


And this in my kalman_filter.cpp:

#include "kalman_filter.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */
KalmanFilter::KalmanFilter() {}
KalmanFilter::~KalmanFilter() {}
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}
void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;  
}
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  MatrixXd Hj = Tools::CalculateJacobian(z);
  VectorXd z_pred = Hj * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;  
  
}

/*

It seems that there is a problem when you try to initialize the state variables in RADAR case. Here the conversion should be done in the other way: the input (i.e. the measurement) is rho, phi and rho-dot and you have to calculate x,y,vx and vy. And no angle-normalization is needed here.

It seems that the transformation (and angle normalization) which was mentioned above is missing from UpdateEKF: here you have to convert x,y,vx,vy into rho, pho and rhodot so that you can calculate the difference between the observed and expected states. (Just think the size of Hj, it is a 3x3 matrix, you cannot multiply it with a vector containing 4 elements.).

I strongly suggest revisiting the videos to get familiarized with the concepts more. This is a natural process, most of the student revisit certain videos many times, especially those which contains new concepts to them.

---

Actually this is more like a Student Hub topic. As you wrote that you do not have access to Student Hub, I keep this Knowledge entry for a while. But it is possible that this will be deleted after a while.


*/