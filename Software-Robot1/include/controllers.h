#ifndef CONTROLLERS_H
#define CONTROLLERS_H

class PIDController {
private:
    double kp;
    double ki;
    double kd;

    double integral;
    double previousError;

public:
    PIDController(double kp, double ki, double kd);

    void setPIDCoefficients(double kp, double ki, double kd);

    // Handle the error calculation outside of this class so it can easily be altered (swapped actual and target values)
    double calculate(double error, double deltaTime);
};

#endif // CONTROLLERS_H
