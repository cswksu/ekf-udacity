#include "tools.h"
#include <iostream>
#include <algorithm>
#include <functional>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {
	/**
	 * TODO: Calculate the RMSE here.
	 */
	std::cout << "calculating RMSE" << std::endl;
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
		return rmse;
	}

	for (int i = 0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - ground_truth[i];

		residual = residual.array()*residual.array();
		rmse += residual;

	}
	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	std::cout << "RMSE successfully calculated" << std::endl;
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
	std::cout << "calculating jacobian" << std::endl;
   MatrixXd jac(3,4);

  if (x_state(0) == 0 && x_state(1)==0) {
	  std::cout << "Divide by zero error in Jacobian Function, returning zeros." << std::endl;
	  jac = MatrixXd::Zero(3, 4);
	  
  }
  jac(0,0)=x_state(0)/pow(pow(x_state(0),2)+pow(x_state(1),2),0.5);
  jac(0,1)=x_state(1)/pow(pow(x_state(0),2)+pow(x_state(1),2),0.5);
  jac(0,2)=0;
  jac(0,3)=0;
  jac(1,0)=-x_state(1)/(pow(x_state(0),2)+pow(x_state(1),2));
  jac(1,1)=x_state(0)/(pow(x_state(0),2)+pow(x_state(1),2));
  jac(1,2)=0;
  jac(1,3)=0;
  jac(2,0)=x_state(1)*(x_state(2)*x_state(1)-x_state(3)*x_state(0))/pow(pow(x_state(0),2)+pow(x_state(1),2),1.5);
  jac(2,1)=x_state(0)*(x_state(3)*x_state(0)-x_state(2)*x_state(1))/pow(pow(x_state(0),2)+pow(x_state(1),2),1.5);
  jac(2,2)=x_state(0)/pow(pow(x_state(0),2)+pow(x_state(1),2),0.5);
  jac(2,3)=x_state(1)/pow(pow(x_state(0),2)+pow(x_state(1),2),0.5);
  
  return jac;
}

