#ifndef PID_H
#define PID_H

#include <vector>

enum State {
  reset,
  add,
  subtract
};

class PID {
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

  // Algorithm buffers
  int iter;
  int initial_steps;
  int tuning_param;
  double total_error;
  double best_error;
  bool tuning_pending;
  bool sim_reset;
  double tolerance;
  State twiddle_state;

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
  void Init(double Kp, double Ki, double Kd, bool twiddle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Implement the Twiddle algo to tune the PID coeff
  */
  void Twiddle();
};

#endif /* PID_H */
