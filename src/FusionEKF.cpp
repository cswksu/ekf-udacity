#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

MatrixXd R_laser_;
MatrixXd R_radar_;
MatrixXd H_laser_;
MatrixXd Hj_;
float noise_ax;
float noise_ay;
KalmanFilter ekf_;
MatrixXd P;
MatrixXd F;
MatrixXd R;
MatrixXd Q;
long long previous_timestamp_;


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
  
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  noise_ax=9.0;
  noise_ay=9.0;

  ekf_=KalmanFilter();
  
  
  P = MatrixXd(4,4);
  F = MatrixXd(4,4);
  R = MatrixXd();
  Q = MatrixXd(4,4);
  F << 1, 0, 1, 0,
	  0, 1, 0, 1,
	  0, 0, 1, 0,
	  0, 0, 0, 1;
  cout << "constructor called" << endl;
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
		 * You'll need to convert radar from polar to cartesian coordinates.
		 */

		 // first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
		ekf_.x_ << 1, 1, 1, 1;
		//previous_timestamp_ = measurement_pack.timestamp_;
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			// TODO: Convert radar from polar to cartesian coordinates 
			//         and initialize state.
			cout << "initializing w/ radar measurement" << endl;
			float rho = measurement_pack.raw_measurements_(0);
			float theta = measurement_pack.raw_measurements_(1);
			float ro_dot = measurement_pack.raw_measurements_(2);

			float x = rho * cos(theta);
			float y = rho * sin(theta);
			float xDot = ro_dot * cos(theta);
			float yDot = ro_dot * sin(theta);
			ekf_.x_ << x, y, xDot, yDot;
			cout << "state variable created for radar" << endl;
			ekf_.P_ << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1000*(1-cos(theta)), 0,
				0, 0, 0, 1000*(1-sin(theta));
			cout << "state covariance variable created for radar" << endl;


		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			// TODO: Initialize state.
			cout << "initializing w/ lidar measurement" << endl;
			float x = measurement_pack.raw_measurements_(0);
			float y = measurement_pack.raw_measurements_(1);

			ekf_.x_(0) = x;
			ekf_.x_(1) = y;
			ekf_.x_(2) = 0;
			ekf_.x_(3) = 0;
			cout << "state variable created for lidar" << endl;
			// done initializing, no need to predict or update
			is_initialized_ = true;
			ekf_.P_ << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1000, 0,
				0, 0, 0, 1000;
			cout << "state covariance variable created for lidar" << endl;
			return;
		}

		cout << "done with initial measurement" << endl;
		
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
	cout << "update timestamp" << endl;
	float dt = -previous_timestamp_;
	cout << dt << endl;

	MatrixXd R_send_;
	MatrixXd H_send_;
	dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	cout << dt << endl;
	previous_timestamp_ = measurement_pack.timestamp_;
	cout << "timestamp and dt updated" << endl;
	cout << "choose correct R and H matrices" << endl;
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		
		R_send_ = R_laser_;
		H_send_ = H_laser_;
		

	}
	else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		R_send_ = R_radar_;
		H_send_ = tools.CalculateJacobian(ekf_.x_);
	}
	cout << "R, H chosen, update F, Q" << endl;
	F << 1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1;
	cout << F << endl;
	cout << "F updated, Q next" << endl;
	Q << pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0,
		0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 3) / 2 * noise_ay,
		pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2)*noise_ax, 0,
		0, (dt, 3) / 2 * noise_ay, 0, pow(dt, 2)*noise_ay;
	cout << "F,Q updated, initialize ekf" << endl;
	ekf_.Init(ekf_.x_, ekf_.P_, F, H_send_, R_send_, Q);
	cout << "initialized, begin prediction step" << endl;
	ekf_.Predict();
	cout << "end prediction step" << endl;

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
		//ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	}
	else {
		// TODO: Laser updates
		cout << "begin lidar update" << endl;
		ekf_.Update(measurement_pack.raw_measurements_);
		cout << "lidar update successful" << endl;
	}

	// print the output
	cout << "prepare to output" << endl;
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
