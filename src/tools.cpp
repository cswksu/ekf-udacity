#include "tools.h"

using Eigen::VectorXd; //using easier namespaces
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {} //constructor

Tools::~Tools() {} //destructor

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {
	
	//initialize rmse and set to zero
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	//sanity check on estimation size and ground truth size
	if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
		return rmse; //return all zeros if non-sensical vector sizes
	}

	//for every estimation
	for (int i = 0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - ground_truth[i]; //calculate residual between estimation and ground truth

		residual = residual.array()*residual.array(); //element-wise square
		rmse += residual;  //rmse becomes sum of all squared residuals

	}
	rmse = rmse / estimations.size(); //divide by number of estimations
	rmse = rmse.array().sqrt();  //square root finishes RMSE
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd jac(3,4);

	if (x_state(0) == 0 && x_state(1)==0) {
		std::cout << "Divide by zero error in Jacobian Function, returning zeros." << std::endl;
		jac = MatrixXd::Zero(3, 4);
	  
	}

	float c1 = pow(x_state(0), 2) + pow(x_state(1), 2);
	float c2 = pow(c1, 0.5);
	float c3 = pow(pow(x_state(0), 2) + pow(x_state(1), 2), 1.5);
	jac(0,0)=x_state(0)/c2;
	jac(0,1)=x_state(1)/c2;
	jac(0,2)=0;
	jac(0,3)=0;
	jac(1,0)=-x_state(1)/c1;
	jac(1,1)=x_state(0)/c1;
	jac(1,2)=0;
	jac(1,3)=0;
	jac(2,0)=x_state(1)*(x_state(2)*x_state(1)-x_state(3)*x_state(0))/c3;
	jac(2,1)=x_state(0)*(x_state(3)*x_state(0)-x_state(2)*x_state(1))/c3;
	jac(2,2)=x_state(0)/c2;
	jac(2,3)=x_state(1)/c2;
  
  return jac;
}

