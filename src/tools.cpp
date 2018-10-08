#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
    int estimation_size = estimations.size();
    int ground_truth_size = ground_truth.size();
    int same_size = estimation_size == ground_truth_size;
	if (estimation_size == 0 or same_size == false)
	{
	    return rmse;
	}
	VectorXd acc(4);
	acc << 0,0,0,0;
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        VectorXd diff = estimations[i] - ground_truth[i];
        VectorXd diff_2 = diff.array() * diff.array();
        acc = acc + diff_2;
	}

	//calculate the mean
	VectorXd mean(4);
	mean << 0,0,0,0;
	mean = acc / estimation_size;

	//calculate the squared root
	rmse = mean.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float px_2 = px * px;
	float py_2 = py * py;
	float c1 = px_2+py_2;
	float dist = sqrt(c1);
	float c3 = (c1*dist);

	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj(0,0) = px / dist;
	Hj(0,1) = py / dist;
	Hj(0,2) = 0;
	Hj(0,3) = 0;

	Hj(1,0) = -(py / (px_2 + py_2));
	Hj(1,1) = px / (px_2 + py_2);
	Hj(1,2) = 0;
	Hj(1,3) = 0;

	Hj(2,0) = (py * (vx*py - vy*px)) / c3;
	Hj(2,1) = (px * (vy*px - vx*py)) / c3;
	Hj(2,2) = px / dist;
	Hj(2,3) = py / dist;

	// from Quiz - compute the Jacobian matrix
	/*Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    */
	return Hj;
}
