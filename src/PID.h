#ifndef PID_H
#define PID_H

#include <vector>

class PID {
    public:
        /**
        * Constructor
        */
        PID();

    
        /**
        * Destructor.
        */
        virtual ~PID();

    
        /**
        * Initialize PID.
        * @param (Kp_, Ki_, Kd_) The initial PID coefficients
        */
        void Init(double Kp_, double Ki_, double Kd_);

    
        /**
        * Update the PID error variables given cross track error.
        * @param cte The current cross track error
        */
        void UpdateError(double cte);

    
        /**
        * Calculate the total PID error.
        * @output The total PID error
        */
        double TotalError();
    
    
        /**
         * AddDpToParam
         * @param idx Index of dp and param currently under scrutiny
         * @param dp_ Value by which param[idx] is adjusted
         */
    void AddDpToParam(int idx, double dp_);
    
     private:
        /**
        * PID Errors
        */
        double p_error;
        double i_error;
        double d_error;
    
    
        double prev_cte;

        /**
        * PID Coefficients
        */
        double Kp;
        double Ki;
        double Kd;
    
        /**
         * Twiddle params
         */
        // number of update steps required until car is up to speed after launching simulator
        // update steps < init_steps don't contribute to reasonable "score", i.e. total error computed over roughly one lap
        int init_steps;
    
        // param currently modified
        int param_idx;
    
        // number of update steps to complete roughly one full lap (depending on throttle value and hardware)
        // necessary to complete a full lap with each changed param to have a meaningful "score" for the parameter set
        int eval_steps;
    
        // number of update steps after changing param before restarting to calculate total_error
        int get_used_steps;
    
        // update_step = step - init_steps
        int update_step;
    
        int step;
        bool twiddle, tried_adding, tried_subtracting;
        double total_error, best_error;
        std::vector<double> dp;
    };

#endif  // PID_H
