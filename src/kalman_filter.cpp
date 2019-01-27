#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd P_, P_prime_, F_, H_, R_, Q_;
VectorXd x_, x_prime_;

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
  VectorXd x_prime_ = F_ * x_;
  MatrixXd P_prime_ = F_ * P_ * F_.transpose()+Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y_ = z-H_*x_prime;
  MatrixXd S_ = H_ * P_prime_ * H_.transpose() + R_;
  MatrixXd K_ = P_prime_ * H_.transpose() * S_.inverse();

  x_=x_prime_ + K_ * y_;
  MatrixXd kh_=K_*h_;
  MatrixXd iden_ = MatrixXd::Identity(kh_.rows(),kh_.cols());
  P_=(iden_-kh_) * P_prime_;




}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
