#include "PID.h"
#include <iostream>
#include <vector>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  sum_cte = 0;

  p_error = 0;
  i_error = 0;
  d_error = 0;
  total_error = 0;
  best_error = 10000.0;

  double p0 = 0.2;
  double p1 = 0;
  double p2 = 4.0;

  dp.push_back(p0);
  dp.push_back(p1);
  dp.push_back(p2);

  p.push_back(Kp_);
  p.push_back(Ki_);
  p.push_back(Kd_);

  p_param_error_high = true;
  i_param_error_high = true;
  d_param_error_high = true;
  
  train_p_param = true;
  train_i_param = false;
  train_d_param = false;

  prev_is_initialized = false;
}

void PID::UpdateError(double cte) {
  if (prev_is_initialized == false) {
    prev_cte = cte;
    prev_is_initialized = true;
  }

  diff_cte = cte - prev_cte;
  sum_cte += cte;

  p_error = cte*Kp_;
  i_error = sum_cte*Ki_;
  d_error = diff_cte*Kd_;

  prev_cte = cte;

  total_error = cte*cte;

}

double PID::TotalError() {
  return(p_error+i_error+d_error);
}
