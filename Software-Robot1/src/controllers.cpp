#include "controllers.h"

PIDController::PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), integral(0.0), previousError(0.0)
{
}

void PIDController::setPIDCoefficients(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

double PIDController::calculate(double error, double deltaTime) {
    integral += error * deltaTime;
    double derivative = (error - previousError) / deltaTime;
    double output = kp * error + ki * integral + kd * derivative;

    previousError = error;
    return output;
}
