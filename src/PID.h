#ifndef PID_H
#define PID_H
#include <iostream>
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double best_error;

  double total_error;

  // parameters
  std::vector<double> p;
  std::vector<double> dp;

  /*
  * Coefficients
  */
  double Kp_;
  double Ki_;
  double Kd_;

  //
  double prev_cte;
  double sum_cte;
  double diff_cte;

  // flags
  bool prev_is_initialized;
  bool p_param_error_high;
  bool i_param_error_high;
  bool d_param_error_high;
  bool train_p_param;
  bool train_i_param;
  bool train_d_param;


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
