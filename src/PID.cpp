#include "PID.h"
#include <iostream>

/**
 * Constructor
 */
PID::PID() {}


/**
 * Destructor.
 */
PID::~PID() {}


/*
 * Initialize PID.
 */
void PID::Init(double Kp_, double Ki_, double Kd_) {
    PID::Kp = Kp_;
    PID::Ki = Ki_;
    PID::Kd = Kd_;
    
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    
    // Previous cte.
    prev_cte = 0.0;
    timestep = 0;
}


/**
 * Update the PID error variables given cross track error.
 * @param cte The current cross track error
 */
void PID::UpdateError(double cte) {
    // Proportional error
    p_error = cte;
    
    // Integral error
    i_error += cte;
    
    // Diferential error
    d_error = cte - prev_cte;
    prev_cte = cte;
    
    // print number of update steps to console to be able to determine length of one lap (in # of update steps)
    // necessary to complete a full lap with each changed param to have a meaningful score for the parameter set
    std::cout << "Timestep: " << update_step << std::endl;
    update_step += 1;
}


/**
 * Calculate the total PID error.
 * @output The total PID error
 */
double PID::TotalError() {
    return p_error * Kp + i_error * Ki + d_error * Kd;
}
