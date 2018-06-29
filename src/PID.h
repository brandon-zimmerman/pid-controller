#ifndef PID_H
#define PID_H

class PID {

/*
* Previous CTE
*/
double previous_cte_;
double int_cte_;
double best_error_;
int total_runs_;
double total_error_;
bool last_error_smaller_than_best_;

double p_[3];
double dp_[3];
int next_adjustment_ = 0;

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Steering angle
  */
  double steer_value;
  
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
