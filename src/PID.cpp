#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  K[0] = Kp_;
  K[1] = Ki_;
  K[2] = Kd_;
  initialized = false;
  int_cte = 0;
  sum_of_squared_errors = 0;
  steps = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  
  if(!initialized) {
    prev_cte = cte;
    initialized = true;
  }
  p_error = cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
  int_cte += cte;
  i_error = int_cte;
  sum_of_squared_errors += cte*cte;
  steps++;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  //std::cout << "p_error:" << p_error << "\t i_error:" << i_error << "\t d_error:" << d_error << std::endl;
  return K[0] * p_error + K[1] * i_error + K[2] * d_error;  // TODO: Add your total error calc here!
}


double PID::MSE() {
  return sum_of_squared_errors / steps;
}

void PID::PrintK() {
  std::cout << "["<< K[0] << ", " << K[1] << ", " << K[2] << "]" << std::endl;
}