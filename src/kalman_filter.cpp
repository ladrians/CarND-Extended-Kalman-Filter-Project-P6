#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
    * predict the state
  */
  x_ = F_ * x_; // section 8 in lesson 5
  MatrixXd Ft = F_.transpose(); // section 9 in lesson 5
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */

  // In section 7 of lesson 5
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  // new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * using data from L5.14
  */
  float x = x_(0);
  float y = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(x*x + y*y);
  if (rho < 0.0001) {
    std::cout << "UpdateEKF(): CalculateJacobian() - Error - Division by Zero" << std::endl;
    rho = 0.0001;
    // return;
  }
  float theta = atan2(y, x);
  float rho_dot = (x*vx + y*vy)/rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;


  VectorXd y_ = z - z_pred;

  // angle normalization
  while (y_(1) > M_PI) {
    y_(1) -= 2 * M_PI;
  }
  while (y_(1) < -M_PI) {
    y_(1) += 2 * M_PI;
  }

  // Section 7 of lesson 5
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y_);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}