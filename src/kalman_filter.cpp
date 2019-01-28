#include "kalman_filter.h"


using Eigen::MatrixXd; //using namespaces from eigen
using Eigen::VectorXd;

MatrixXd P_prime_; //object variables
VectorXd x_prime_;



KalmanFilter::KalmanFilter() { //constructor
	x_ = VectorXd(4); //initialize x vector
	P_ = MatrixXd(4, 4);  // initialize P matrix
}

KalmanFilter::~KalmanFilter() {} //destructor

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in; //take in matrices and vectors from inputs
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() {
	x_prime_ = F_ * x_; //calculate predicted x
	P_prime_ = F_ * P_ * F_.transpose()+Q_; //calculate predicted P
}

void KalmanFilter::Update(const VectorXd &z) { //non-extended KF
	VectorXd y_ = z-H_*x_prime_; //calculate error in prediction
	MatrixXd S_ = H_ * P_prime_ * H_.transpose() + R_; //calcluate S
	MatrixXd K_ = P_prime_ * H_.transpose() * S_.inverse(); //calculate K
	x_=x_prime_ + K_ * y_; //calculate updated x
	MatrixXd kh_=K_*H_; //calculate K*H
	MatrixXd iden_ = MatrixXd::Identity(P_.rows(),P_.cols()); //create properly sized identity matrix
	P_=(iden_-kh_) * P_prime_; //calcluate updated P
}

void KalmanFilter::UpdateEKF(const VectorXd &z) { //extended KF
	VectorXd h_ = VectorXd(3); //calculate h(x')
	if (x_prime_(0) != 0 && x_prime_(1) != 0) { //if x and y are both non-zero
		h_(0) = sqrt(pow(x_prime_(0), 2) + pow(x_prime_(1), 2)); //calculate rho
		h_(1) = atan2(x_prime_(1), x_prime_(0)); //phi
		h_(2) = (x_prime_(0)*x_prime_(2) + x_prime_(1)*x_prime_(3)) / h_(0); //rho dot
	}
	else { //both zero case
		std::cout << "Divide by Zero in UpdateEKF, returning h(x_prime) = 0" << std::endl;
		h_ << 0, 0, 0;  //output all zeros
	}

	
	VectorXd y_ = z - h_; //error calculation
	if (y_(1) < -M_PI) { //if error is less than -pi
		while (y_(1) < M_PI) { //while still less than -pi
			y_(1) += 2 * M_PI; //add 2pi
		}
	}
	else if (y_(1) > M_PI) { //see above, but for >+pi case
		while (y_(1) > M_PI) {
			y_(1) -= 2 * M_PI;
		}
	}
	MatrixXd S_ = H_ * P_prime_ * H_.transpose() + R_; //calculate S
	MatrixXd K_ = P_prime_ * H_.transpose() * S_.inverse(); //calculate K

	x_ = x_prime_ + K_ * y_; //calculate updated x
	MatrixXd kh_ = K_ * H_; //calculate K*H
	MatrixXd iden_ = MatrixXd::Identity(kh_.rows(), kh_.cols()); //generate properly sized identity matrix
	P_ = (iden_ - kh_) * P_prime_; //calculate updated P
}
