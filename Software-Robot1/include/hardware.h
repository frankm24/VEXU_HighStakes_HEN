#ifndef HARDWARE_H
#define HARDWARE_H

#include "vex.h"

using namespace vex;

class Drivetrain {
    private:
        const motor front_left_motor;
        const motor back_left_motor;
        const motor back_right_motor;
        const motor front_right_motor; 
        const encoder left_encoder;
        const encoder right_encoder;
        const encoder perpendicular_encoder;
    public:
        Drivetrain();
};

#endif