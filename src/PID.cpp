#include "PID.h"
#include "math.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

const double MAX_STEER = M_PI / 4.0;

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	
	p_[0] = Kp;
	p_[1] = Ki;
	p_[2] = Kd;

	dp_[0] = 1;
	dp_[1] = 1;
	dp_[2] = 1;

	last_error_smaller_than_best_ = true;

	previous_cte_ = 0;
	next_adjustment_ = 0;
	total_error_ = 0;
	total_runs_ = 0;
	best_error_ = 0;
	int_cte_ = 0;

	p_[next_adjustment_] += dp_[next_adjustment_];

}

void PID::UpdateError(double cte) 
{
	total_runs_++;
	total_error_ += pow(cte, 2);	

	if (best_error_ == 0)
	{
		best_error_ = TotalError();
	}

	double dsum = 0.0;

	for(unsigned int i=0; i<3; i++)	
	{
		dsum += dp_[i];
	}

	if(dsum>0.00001)
	{ 
		if (last_error_smaller_than_best_)
		{
			if (TotalError() < best_error_)
			{
				best_error_ = TotalError();
				dp_[next_adjustment_] *= 1.1;
				last_error_smaller_than_best_ = true;
			}
			else
			{
				p_[next_adjustment_] -= 2 * dp_[next_adjustment_];
				last_error_smaller_than_best_ = false;
			}
		}
		else
		{
			if (TotalError() < best_error_)
			{
				best_error_ = TotalError();
				dp_[next_adjustment_] *= 1.1;
			}
			else
			{
				p_[next_adjustment_] += dp_[next_adjustment_];
				dp_[next_adjustment_] *= 0.9;
			}
		}

		if (next_adjustment_ == 2)
			next_adjustment_ = 0;
		else
			next_adjustment_++;

	}
	double diff_cte = cte - previous_cte_;
	previous_cte_ = cte;
	int_cte_ += cte;
	
	double steer = -p_[0] * cte - p_[1] * diff_cte - p_[2] * int_cte_;

	if (steer > MAX_STEER)
		steer = MAX_STEER;

	if (steer < -MAX_STEER)
		steer = -MAX_STEER;

	this->steer_value = steer;


}


double PID::TotalError() {

	return this->total_error_ / total_runs_;
}

