#ifndef PATHS_H
#define PATHS_H

#include "vex.h"
#include "odometry.h"

// Follow a straight line. Don't underestimate the complexity of this task. 
// A straight line is a very precise thing. 

struct Point {
    const double x;
    const double y;
};

struct LineSegment {
    const Point p1;
    const Point p2;
};

class TrapezoidalProfile {
    private:
        enum ProfileType {
            TRAPEZOID,
            TRIANGLE
        };
        const ProfileType PROFILE_TYPE;
        const double MAX_ACCELERATION;
        const double MAX_VELOCITY;
        const double D_CRITICAL;
        const double D;
        double DELTA_T_ACCEL;
        double DIST_ACCEL;
        double V_PEAK = 0, T_1 = 0, T_2 = 0, T_3 = 0;

    public: 
        TrapezoidalProfile(double distance, double MAX_ACCELERATION, 
        double MAX_VELOCITY);
        double calculateVelocity(double time);
        double getDuration();
};

namespace PathFollowing {
    void driveForward(double distance, ThreeWheelLocalizer localizer, 
    OdometryConstants odometry_constants, vex::motor_group left_motor_group,
    vex::motor_group right_motor_group);

    void rotate(double degrees);
}

#endif