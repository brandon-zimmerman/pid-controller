#include "PID.h"
#include "math.h"
#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

const double MAX_STEER = M_PI / 4.0;

void PID::Init(double Kp, double Ki, double Kd) {

	// coefficients
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	
	// error values
	p_error_ = 0;
	i_error_ = 0;
	d_error_ = 0;	

	// twiddle values
	dp_[0] = .001;
	dp_[1] = 0.0001;
	dp_[2] = .001;	
	last_error_smaller_than_best_ = true;	
	next_term_ = Proproportional;
	update_count_ = 0;
	error_ = 0;	
	best_error_ = 0;
	twiddle_ = false;
	
	// last total error
	last_total_error_ = 0;

	// time of last update
	last_update_time_ = steady_clock::now();
}


void PID::UpdateError(double cte)
{
	// calc time since last update
	steady_clock::time_point now = steady_clock::now();	
	double duration = duration_cast<std::chrono::milliseconds>(now - last_update_time_).count() / 1000;	

	if (duration == 0) duration = 1;

	// increment the run count.
	update_count_++;	

	// adjust tuning parameters
	double kp = this->Kp;
	double ki = this->Ki * duration;
	double kd = this->Kd / duration;

	// calculate the error values
	d_error_ = cte - p_error_;
	i_error_ += cte;
	p_error_ = cte;

	// calculate the total error 
	last_total_error_ = -p_error_ * kp - i_error_ * ki - d_error_ * kd;

	//cout << "p_error_=" << p_error_ << "|i_error_=" << i_error_ << "|d_error_=" << d_error_<<std::endl;

	// update the last update time
	last_update_time_ = steady_clock::now();

	// check if twiddle in enabled
	if (twiddle_)
	{		
		error_ += pow(cte, 2);

		double error = error_ / update_count_;

		update_count_ = 0;

		if (best_error_ == 0) best_error_ = error;

		double error_sum = dp_[0] + dp_[1] + dp_[2];

		double term_value;
		switch (next_term_)
		{
		case Proproportional:
			term_value = this->Kp;
			break;
		case Integral:
			term_value = this->Ki;
			break;
		default:
			term_value = this->Kd;
			break;
		}


		if (error_sum > 0.00001)
		{
			if (last_error_smaller_than_best_)
			{
				if (error < best_error_)
				{
					best_error_ = error;
					dp_[next_term_] *= 1.1;
					last_error_smaller_than_best_ = true;
				}
				else
				{
					term_value -= 2 * dp_[next_term_];
					last_error_smaller_than_best_ = false;
				}
			}
			else
			{
				if (error < best_error_)
				{
					best_error_ = error;
					dp_[next_term_] *= 1.1;
				}
				else
				{
					term_value += dp_[next_term_];
					dp_[next_term_] *= 0.9;
				}
			}

			switch (next_term_)
			{
			case Proproportional:
				this->Kp = term_value;
				next_term_ = Integral;
				break;
			case Integral:
				this->Ki = term_value;
				next_term_ = Derivative;
				break;
			default:
				this->Kd = term_value;
				next_term_ = Proproportional;
				break;
			}
		}		
	}	
}


double PID::TotalError() {

	return last_total_error_;
}

