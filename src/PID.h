#ifndef PID_H
#define PID_H

#include <array> // Added by the student

class PID 
{
    public:
        PID();
        virtual ~PID();

        // Initialize PID.
        void Init(double Kp, double Kd, double Ki);

        // Update the PID error variables given cross track error.
        void UpdateError(double cte);

        // Calculate the total PID error.
        double TotalError();

        void Twiddle(); // Method added by the student



    private:
        // Errors
        double p_error_;
        double i_error_;
        double d_error_;
        double twiddle_error_;  // error value for twiddle parameter optimization

        // Coefficients
        std::array<double, 3> params_; // Kp, Kd, Ki
        std::array<double, 3> dparams_; // Twiddle parameters       

        int cycle;

        int param_counter; 
        
};

#endif /* PID_H */
