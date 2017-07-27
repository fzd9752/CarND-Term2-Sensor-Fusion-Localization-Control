#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Parameters for twiddle
  */
  bool is_twiddle;
  bool is_Init;

  std::vector<double> dp;

  double total;
  double best_error;
  double total_error;
  double error;

  int step;
  int start_step;
  int end_step;

  bool flag_inc;
  bool flag_fist;
  bool finished;

  int i;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp_;
  double Ki_;
  double Kd_;

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

  /*
  * Control speed.
  */
  double Thtottle(double steer_value);

  /*
  * Funciton for twiddle
  */
  void twiddleInit(double tol, int start, int end);

  void twiddle();

private:
  /*
  * helpful function for twiddle
  */
  double dpSum(std::vector<double> x);
};

#endif /* PID_H */
