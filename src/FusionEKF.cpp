#include "FusionEKF.h"

//using appropriate namespaces
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

//set object variables
KalmanFilter ekf_;
MatrixXd P;
MatrixXd F;
MatrixXd R;
MatrixXd Q;
long long previous_timestamp_;


//set noise parameters
#define noise_ax 9.0
#define noise_ay 9.0

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false; //1st measurement has not been taken yet

	previous_timestamp_ = 0.0; //initialize previous timestamp

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0.0,
		0.0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0.0, 0.0,
		0.0, 0.0009, 0.0,
		0.0, 0.0, 0.09;
  
	//H matrix for laser measurements
	H_laser_ << 1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0;

	//initialize ekf
	ekf_=KalmanFilter();
  
	//initialize matrices
	P = MatrixXd(4,4);
	F = MatrixXd(4,4);
	Q = MatrixXd(4,4);
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
		// first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4); //initialize state variable
		ekf_.x_ << 1.0, 1.0, 1.0, 1.0; //set dummy values for ekf.x
		previous_timestamp_ = measurement_pack.timestamp_; //set previous timestampt as 1st measurement
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			// If radar 1st measurement

			// pull measurements values from measurement pack
			float rho = measurement_pack.raw_measurements_(0);
			float theta = measurement_pack.raw_measurements_(1);
			float ro_dot = measurement_pack.raw_measurements_(2);

			//turn into cartesian coordinates
			float x = rho * cos(theta);
			float y = rho * sin(theta);

			//while we don't get velocity exactly, we can get component of velocity along radar vector
			float xDot = ro_dot * cos(theta);
			float yDot = ro_dot * sin(theta);
			
			//create x vector, p matrix
			ekf_.x_ << x, y, xDot, yDot;
			ekf_.P_ << 1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 1000.0*(sin(theta)), 0.0,
				0.0, 0.0, 0.0, 1000.0*(cos(theta));
			
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			// If lidar first measurment

			// pull x and y from measurement pack
			float x = measurement_pack.raw_measurements_(0);
			float y = measurement_pack.raw_measurements_(1);

			// initialize state vector
			ekf_.x_(0) = x;
			ekf_.x_(1) = y;
			ekf_.x_(2) = 0.0;
			ekf_.x_(3) = 0.0;
			
			//initialize P matrix
			ekf_.P_ << 1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 1000.0, 0.0,
				0.0, 0.0, 0.0, 1000.0;
		}
		is_initialized_ = true;
		// done initializing, no need to predict or update
		return;
	}

	/**
	 * Prediction
	 */
	
	//Will need to choose which R and H to send to init function
	MatrixXd R_send_;
	MatrixXd H_send_;
	
	//calculate delta t, set new time to old time
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		//if radar, set appropriate R, H matrices
		R_send_ = R_radar_;
		H_send_ = tools.CalculateJacobian(ekf_.x_); //Hj
	}

	else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		//if lidar data pack, set appropriate R, H
		R_send_ = R_laser_;
		H_send_ = H_laser_;
	}
	
	//use dt to update state transition matrix
	F << 1.0, 0.0, dt, 0.0,
		0.0, 1.0, 0.0, dt,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0;

	float dt4 = pow(dt, 4.0)/4.0;  //precalculate powers of dt
	float dt3 = pow(dt, 3.0)/2.0;
	float dt2 = pow(dt, 2.0);

	//calculate Q matrix
	Q << dt4 * noise_ax, 0.0, dt3* noise_ax, 0.0,
		0.0, dt4 * noise_ay, 0.0, dt3 * noise_ay,
		dt3 * noise_ax, 0.0, dt2*noise_ax, 0.0,
		0.0, dt3 * noise_ay, 0.0, dt2*noise_ay;
	
	//Initialize kalman filter object
	ekf_.Init(ekf_.x_, ekf_.P_, F, H_send_, R_send_, Q);
	
	//Predict using F
	ekf_.Predict();
	
	/**
	 * Update
	 */

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	}
	else {
		// TODO: Laser updates
		ekf_.Update(measurement_pack.raw_measurements_);
	}

	// print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
