#include "PID.h"
#include <math.h>
#include <cmath>
#include <iostream>

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

  d_error = 99999.9;

  step = 0;
}

void PID::UpdateError(double cte) {
  if (d_error == 99999.9)
  {
    p_error = cte;
  }
  d_error = cte - p_error;
  //cout << "Different Error: " << d_error << endl;
  p_error = cte;
  i_error += cte;
  if (cte == 0 ) {
    i_error = 0;
  }

  step += 1;
  if (is_twiddle && step >= start_step){
    total_error += pow(cte, 2);
  }

}

double PID::TotalError() {
  return - Kp_ * p_error - Ki_ * i_error - Kd_ * d_error;
}

double PID::Thtottle(double steer_value){
  double throttle_value = (1.0 - fabs(steer_value)) * fabs(p_error) * Kp_;

  if (throttle_value >= 1.0) {
    return 1.0;
  } else if (throttle_value < 0.1) {
    return 0.1;
  } else {
    return throttle_value;
  }

}

/*
 * TODO: Complete twittdle part
 */


void PID::twiddleInit(double tol, int start, int end) {
  total = tol;
  start_step = start;
  end_step = end;

  is_Init = false;
  is_twiddle = true;
  total_error = 0.0;

  dp = {0.015, .2, 0.00005};
  flag_inc = true;
  flag_fist = true;
  finished = false;

  i = 0;
}

void PID::twiddle() {
  if (step > end_step) {
    error = total_error / (step - start_step);

    cout << "Best Error = " << best_error << endl;
    cout << "Error = " << error << endl;

    cout << "Dp_Kp = " << dp[0] << " Dp_Kd = "
    << dp[1] << " Dp_Ki = " << dp[2] << '\n';

    cout << "Kp = " << Kp_ << " Kd = "
    << Kd_ << " Ki = " << Ki_ << '\n';

    cout << "Dp Sum = " << dpSum(dp) << endl;

    if (!is_Init) {
      // The first lap always drive less distance
      // than the later lap with same steps, so
      // I ignore the error for the first lap
      best_error = 10.0;
      is_Init = true;
    } else {
      if (flag_fist){
        switch(i){
          case 0:
            Kp_ += dp[i];
            break;
          case 1:
            Kd_ += dp[i];
            break;
          case 2:
            Ki_ += dp[i];
        }
        flag_fist = false;
        flag_inc = true;
      } else if (error < best_error) {
        best_error = error;
        if (dpSum(dp) < total) {
          finished = true;
          cout << "Twiddle Complete." << endl;
          cout << "Error = " << error << endl;
          cout << "Kp = " << Kp_ << " Kd = "
          << Kd_ << " Ki = " << Ki_ << '\n';
        }
        dp[i] *= 1.1;
        i = (i+1) % 3;
        flag_fist = true;
      } else if (flag_inc) {
        switch(i){
          case 0:
            Kp_ -= 2 * dp[i];
            break;
          case 1:
            Kd_ -= 2 * dp[i];
            break;
          case 2:
            Ki_ -= 2 * dp[i];
        }
        flag_inc = false;
      } else {
        switch(i){
          case 0:
            Kp_ += dp[i];
            break;
          case 1:
            Kd_ += dp[i];
            break;
          case 2:
            Ki_ += dp[i];
        }
        dp[i] *= 0.9;
        i = (i+1) % 3;
        flag_fist = true;
      }

    }
    step = 0;
    total_error = 0;

  }

}

double PID::dpSum(vector<double> x) {
  double t = 0.0;
  for (unsigned i=0; i < x.size(); i++){
    t += x[i];
  }
  return t;
}
