#include "PID.h"
#include <iostream>
#include <cfloat>
#include <limits>
#include <numeric>
#include <iomanip> 

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Cp, double Ci, double Cd, bool twiddle) {
  Kp = Cp;
  Ki = Ci;
  Kd = Cd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  iter = 0;
  total_error = 0;
  best_error = DBL_MAX;
  tuning_pending = twiddle;
  sim_reset = false;
  initial_steps = 1000;

  // dp = {1, 1, 1};
  tolerance = 0.006;
  tuning_param = 0;
  twiddle_state = State::reset;
}

void PID::UpdateError(double cte) {
  iter++;
  sim_reset = false;
  if(iter == 1) {
    // 1st iteration, save the cte as p_error
    p_error = cte;
  }

  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;

  // calculate error after completion of initial_steps
  if (iter >= initial_steps) {
    total_error += cte * cte;
  }

  total_error /= iter;

  // total_error /= iter;

  // cout << " p_error = " << p_error << " i_error = " << i_error << " d_error = " << d_error << endl;

  if(tuning_pending){
    if(iter > 2 * initial_steps && 
        TotalError() > tolerance) {
      // if(total_error < best_error)
      //   best_error = total_error;
      Twiddle();
      cout << "Total error in this try = " << total_error << " best error " << best_error << endl;
      iter = 0;
      total_error = 0;
      i_error = 0;
      sim_reset = true;
      // cout << "dp_p = " << setprecision(3) << dp[0] << " dp_i = " << setprecision(3) << dp[1] << " dp_d = " << setprecision(3) << dp[2] << endl;
    }
    // cout << " accum error = " << dp[0] + dp[1] + dp[2] << endl;
    if(TotalError() <= tolerance) {
      // cout.setprecision(3);
      cout << "Kp = " << setprecision(3) << Kp << " Ki = " << setprecision(3) << Ki << " Kd = " << setprecision(3) << Kd << endl;
      cout << "dp_p = " << setprecision(3) << p_error << " dp_i = " << setprecision(3) << i_error << " dp_d = " << setprecision(3) << d_error << endl;
      cout << "Total error = " << TotalError()  << endl;
      tuning_pending = false;
      sim_reset = true;
      i_error = 0;
      total_error = 0;
      // exit(0);
    }
  }
}


void PID::Twiddle() {  
  std::vector<double> p = {Kp, Ki, Kd};
  std::vector<double> dp = {p_error, i_error, d_error};
  cout << "total error " << total_error << endl;
  switch(twiddle_state) {
    case State::reset:
      p[tuning_param] += dp[tuning_param];
      twiddle_state = State::add;
      cout << "ADD" << endl;
    break;

    case State::add:
      if(total_error < best_error) {
        best_error = total_error;
        dp[tuning_param] *= 1.1;
        cout << "add error improvement. dp " << dp[tuning_param] << " param " << tuning_param << endl;
        tuning_param = (tuning_param + 1) % dp.size();
        twiddle_state = State::reset;
      }
      else {
        p[tuning_param] -= 2 * dp[tuning_param];
        twiddle_state = State::subtract;
        cout << "SUB" << endl;
      }
    break;

    case State::subtract:
      if(total_error < best_error) {
        best_error = total_error;
        dp[tuning_param] *= 1.1;
        cout << "sub error improvement. dp " << dp[tuning_param] << " param " << tuning_param << endl;
        tuning_param = (tuning_param + 1) % dp.size();
        twiddle_state = State::reset;
      }
      else {
        p[tuning_param] += dp[tuning_param];
        dp[tuning_param] *= 0.9;
        cout << "reduce error improvement. dp " << dp[tuning_param] << " param " << tuning_param << endl;
        tuning_param = (tuning_param + 1) % dp.size();
        twiddle_state = State::reset;
      }
    break;
  }

  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
  p_error = dp[0];
  i_error = dp[1];
  d_error = dp[2];
  cout << "Twiddle Kp = " << setprecision(3) << Kp << " Ki = " << setprecision(3) << Ki << " Kd = " << setprecision(3) << Kd << endl;
}

double PID::TotalError() {
  return p_error + d_error + i_error;
}

