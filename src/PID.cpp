#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

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
    
    prev_cte = 0.0;
    
    // Twiddle params
    twiddle = true;
    step = 1;
    param_idx = 2;
    init_steps = 900;
    eval_steps = 1200; // number of update steps to complete roughly one full lap at throttle 0.7 (once car is up to speed) on my Mac
    get_used_steps = 100; 
    tried_adding = false;
    tried_subtracting = false;
    total_error = 0;
    best_error = std::numeric_limits<double>::max();
    dp = {0.1*Kp, 0.1*Ki, 0.1*Kd};
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
    
    
    // Twiddle algorithm (https://www.youtube.com/watch?v=2uQ2BSzDvXs)
    if (step > init_steps) {
        update_step = step - init_steps;
        
        if (update_step % (get_used_steps + eval_steps) > get_used_steps){
            total_error += pow(cte,2);
        }
        
        if (twiddle && update_step % (get_used_steps + eval_steps) == 0){
            cout << "update step: " << update_step << endl;
            cout << "total error: " << total_error << endl;
            cout << "best error: " << best_error << endl;
            if (total_error < best_error) {
                cout << "TOTAL ERROR < BEST ERROR" << endl;
                best_error = total_error;
                if (update_step !=  get_used_steps + eval_steps) {
                    // only to be executed once adding or subtracting were performed once
                    dp[param_idx] *= 1.1;
                }
                // continue with next parameter
                param_idx = (param_idx + 1) % 3;
                tried_adding = tried_subtracting = false;
            }
            
            if (!tried_adding && !tried_subtracting) {
                // try adding dp[i] to params[i]
                AddDpToParam(param_idx, dp[param_idx]);
                tried_adding = true;
            }
            
            else if (tried_adding && !tried_subtracting) {
                // try subtracting dp[i] from params[i]
                AddDpToParam(param_idx, -2 * dp[param_idx]);
                tried_subtracting = true;
            }
            
            else {
                // reduce dp[i]
                AddDpToParam(param_idx, dp[param_idx]);
                dp[param_idx] *= 0.9;
                // continue with next parameter
                param_idx = (param_idx + 1) % 3;
                tried_adding = tried_subtracting = false;
            }
            
            total_error = 0;
            cout << "UPDATED PARAMETERS" << endl;
            cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;
        }
    }
    // TWIDDLE PREP
    // print number of update steps to console to be able to determine length of one lap (in # of update steps)
    // std::cout << "Timestep: " << step << std::endl;
    step++;
}


/**
 * Calculate the total PID error.
 * @output The total PID error
 */
double PID::TotalError() {
    return p_error * Kp + i_error * Ki + d_error * Kd;
}


/**
 * AddDpToParam
 * @param idx Index of dp and param currently under scrutiny
 * @param dp_ Value by which param[idx] is adjusted
 */
void PID::AddDpToParam(int idx, double dp_){
    if (idx == 0) {
        Kp += dp_;
    }
    else if (idx == 1) {
        Ki += dp_;
    }
    else if (idx == 2) {
        Kd += dp_;
    }
    else {
        cout << "AddDpToParam: Index out of bounds";
    }
}
