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
	float rmse = 0;
	VectorXd rmseV(estimations.size());

	for (int i = 0; i < estimations.size(); ++i) {
		rmse += pow(estimations[i] - ground_truth[i], 2);
		rmseV[i] = pow(rmse / i, 0.5);

		return rmseV;
	}
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  jac=MatrixXd(3,4);
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

