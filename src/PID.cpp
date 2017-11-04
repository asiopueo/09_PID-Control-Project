#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	p_error_ = 0.0f;
	d_error_ = 0.0f;
	i_error_ = 0.0f;
}

PID::~PID() {}




void PID::Init(double Kp, double Ki, double Kd) 
{
	Kp_ = Kp;
	Kd_ = Kd;
	Ki_ = Ki;
}

void PID::UpdateError(double cte) 
{
	d_error_ = cte - p_error_; // diff_cte
	p_error_ = cte;
    i_error_ = i_error_ + cte; // int_cte
}

double PID::TotalError() 
{
	double steering;
	steering = - Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;

	cout << "Kp: " << Kp_ << "\t p_error: " << p_error_ << "\t Kd: " 
		<< Kd_ << "\t d_error: " << d_error_ << "\t Ki: " << Ki_ << "\t i_error: " << i_error_ << endl;

	if (steering >= 1.0f)
		return 1.0f;
	else if (steering <= -1.0f)
		return -1.0f;
	else
		return steering;
}

