#ifndef PID_H
#define PID_H

class PID
{
public:
    // Constructor
    PID(const double kp, const double ki, const double kd);

    // Destructor
    virtual ~PID();

    // Update the PID error variables given cross track error
    void updateError(double cte);

    // Calculate the PID output
    double computeSteering();

private:
    // Coefficients
    const double kp_;
    const double ki_;
    const double kd_;

    // Errors
    double p_error_;
    double d_error_;
    double i_error_;

    static constexpr double kSteeringMax = 1.0;
    static constexpr double kSteeringMin = -1.0;
};

#endif /* PID_H */
