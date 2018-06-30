#include <chrono>

#ifndef PID_H
#define PID_H

using namespace std;
using namespace std::chrono;

class PID {


enum ControlTerm { Proproportional = 0, Integral = 1, Derivative = 2 };

/*
* Previous CTE
*/
double best_error_;
int update_count_;
double error_;
bool last_error_smaller_than_best_;

steady_clock::time_point last_update_time_;

double dp_[3];

ControlTerm next_term_ = Proproportional;

bool twiddle_;

const int twiddle_runs_per_pass = 100;

double last_total_error_;



/*
* Errors
*/
double p_error_;
double i_error_;
double d_error_;



public:


  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
    
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);  

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
