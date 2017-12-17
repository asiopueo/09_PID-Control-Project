#include "PID.h"
#include <iostream>
#include <array>  // Added by the student
#include <math.h> // Added by the student

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	p_error_ = 0.0f;
	d_error_ = 0.0f;
	i_error_ = 0.0f;

	twiddle_error_ = 0.0;  // error value for twiddle parameter optimization

	cycle = 0;
	param_counter = 0;
}

PID::~PID() {}




void PID::Init(double Kp, double Kd, double Ki) 
{
	params_ = {Kp, Kd, Ki};
	dparams_ = {1.0, 1.0, 1.0};
	twiddle_error_ = 0.0;
}

void PID::UpdateError(double cte) 
{
	d_error_ = cte - p_error_; // diff_cte
	p_error_ = cte;
    i_error_ = i_error_ + cte; // int_cte

    twiddle_error_ += pow(cte, 2);
    //std::cout << twiddle_error_ << std::endl;
}




double PID::TotalError() 
{
	double steering;
	steering = - params_[0] * p_error_ - params_[1] * d_error_ - params_[2] * i_error_;

	// Uncomment for debugging purposes:
	//cout << "Kp: " << Kp_ << "\t p_error: " << p_error_ << "\t Kd: " << Kd_ << "\t d_error: " << d_error_ << "\t Ki: " << Ki_ << "\t i_error: " << i_error_ << endl;



	if (steering >= 1.0f)
		return 1.0f;
	else if (steering <= -1.0f)
		return -1.0f;
	else
		return steering;
}



void PID::Twiddle()
{
	// Hier besteht noch signifikanter Verbesserungsbedarf!
    double best_err = 100;
    double err = 0;

    double diff_sum = dparams_[0] + dparams_[1] + dparams_[2];

    cycle++;
    param_counter++;


    if (cycle%200==0)
    {
            params_[param_counter%3] += dparams_[param_counter%3];
            
            std::cout << param_counter%3 << std::endl;

            if (err < best_err)
            {
                best_err = err;
                dparams_[param_counter%3] *= 1.1;
                // RESET
            }
            else
            {
                params_[param_counter%3] -= 2*dparams_[param_counter%3];

                if (err < best_err)
                {
                    best_err = err;
                    dparams_[param_counter%3] *= 1.1; 
                    // RESET
                }
                else 
                {
                    params_[param_counter%3] += dparams_[param_counter%3];
                    dparams_[param_counter%3] *= 0.9;
                    // RESET
                }
            }
    //std::string reset_msg = "42[\"reset\",{}]";
    //ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
    }
}




