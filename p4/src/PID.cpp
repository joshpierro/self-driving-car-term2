#include "PID.h"
#include <iostream>
#include <vector>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd,const char* type) {

  //initialize PID Controller with coefficients
  //and set error
  Kp = kp;
  Ki = ki;
  Kd = kd;
  p_error = 0;
  i_error = 0;
  d_error = 0;

  cout << "Initializing " << type << " PID Controller with" << endl;
  cout << " P:" << Kp << endl;
  cout << " I:" << Ki << endl;
  cout << " D:" << Kd << endl;
  cout << "and 0 Error" << endl;

}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
  return (Kp*p_error + Ki*i_error + Kd*d_error)*-1;
}

/*DEPRICATED*/
void PID::Twiddle(){
}
