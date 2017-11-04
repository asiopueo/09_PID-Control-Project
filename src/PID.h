#ifndef PID_H
#define PID_H

class PID 
{
    public:
        // Errors
        double p_error_;
        double i_error_;
        double d_error_;

        // Coefficients
        double Kp_;
        double Ki_;
        double Kd_;

        PID();
        virtual ~PID();

        // Initialize PID.
        void Init(double Kp, double Ki, double Kd);

        // Update the PID error variables given cross track error.
        void UpdateError(double cte);

        // Calculate the total PID error.
        double TotalError();
};

#endif /* PID_H */
