#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    
    p_error = i_error = d_error = 0.0;
    
    cte_prev = 0.0;
    
}

void PID::UpdateError(double cte) {
    cte_prev = p_error;
    
    p_error = cte;
    i_error += cte;
    d_error = (cte - cte_prev);
    
}

double PID::TotalError() {
    return -Kp * p_error - Kd * d_error - Ki * i_error;
}

