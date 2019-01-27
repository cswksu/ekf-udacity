#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd P_, P_prime_, F_, H_, R_, Q_;
VectorXd x_, x_prime_;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
	x_ = VectorXd(4);
	P_ = MatrixXd(4, 4);
}

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
  x_prime_ = F_ * x_;
  std::cout << x_prime_.size() << std::cout;
  P_prime_ = F_ * P_ * F_.transpose()+Q_;
  std::cout << P_prime_.size() << std::cout;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  std::cout << z.size() << std::cout;
  VectorXd y_ = z-H_*x_prime_;
  std::cout << "y calculated" << std::endl;
  MatrixXd S_ = H_ * P_prime_ * H_.transpose() + R_;
  std::cout << "y calculated" << std::endl;
  MatrixXd K_ = P_prime_ * H_.transpose() * S_.inverse();
  std::cout << "K calculated" << std::endl;
  x_=x_prime_ + K_ * y_;

  std::cout << "x calculated" << std::endl;
  MatrixXd kh_=K_*H_;
  std::cout << "kh" << std::endl;
  MatrixXd iden_ = MatrixXd::Identity(kh_.rows(),kh_.cols());
  std::cout << "idendity matrix calculated" << std::endl;
  P_=(iden_-kh_) * P_prime_;
  std::cout << "P prime calculated" << std::endl;




}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

	VectorXd h_ = VectorXd(3);
	if (x_prime_(0) != 0 and x_prime_(1) != 0) {
		h_(0) = sqrt(pow(x_prime_(0), 2) + pow(x_prime_(1), 2));
		h_(1) = atan2(x_prime_(1), x_prime_(0));
		h_(2) = (x_prime_(0)*x_prime_(2) + x_prime_(1)*x_prime_(3)) / h_(0);
	}
	else {
		std::cout << "Divide by Zero in UpdateEKF, returning h(x_prime) = 0" << std::endl;
		h_ << 0, 0, 0;
	}

	VectorXd y_ = z - h_;
	MatrixXd S_ = H_ * P_prime_ * H_.transpose() + R_;
	MatrixXd K_ = P_prime_ * H_.transpose() * S_.inverse();

	x_ = x_prime_ + K_ * y_;
	MatrixXd kh_ = K_ * H_;
	MatrixXd iden_ = MatrixXd::Identity(kh_.rows(), kh_.cols());
	P_ = (iden_ - kh_) * P_prime_;
}
