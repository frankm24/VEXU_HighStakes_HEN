#include "path_planning.h"
#include "cmath"

TrapezoidalProfile::TrapezoidalProfile(double distance, double MAX_ACCELERATION, 
double MAX_VELOCITY) : 
    D(distance),
    D_CRITICAL( pow(MAX_VELOCITY, 2) / MAX_ACCELERATION ),
    MAX_ACCELERATION(MAX_ACCELERATION),
    MAX_VELOCITY(MAX_VELOCITY),
    PROFILE_TYPE( (D >= D_CRITICAL) ? TRAPEZOID : TRIANGLE ),
    DIST_ACCEL(pow(MAX_VELOCITY, 2) / (2 * MAX_ACCELERATION))
{
    if (PROFILE_TYPE == TRAPEZOID) {
        DELTA_T_ACCEL = MAX_VELOCITY / MAX_ACCELERATION;
        T_1 = DELTA_T_ACCEL;
        T_2 = T_1 + (D - 2*DIST_ACCEL) / MAX_VELOCITY;
        T_3 = T_2 + DELTA_T_ACCEL;
    } else {
        V_PEAK = sqrt(D * MAX_ACCELERATION);
        DELTA_T_ACCEL = V_PEAK / MAX_ACCELERATION;
        T_1 = DELTA_T_ACCEL;
        T_2 = T_1;
        T_3 = T_2 + DELTA_T_ACCEL;
    }
}

double TrapezoidalProfile::calculateVelocity(double time) {
    if (PROFILE_TYPE == TRAPEZOID) {
        if (time < T_1) {
            return MAX_ACCELERATION * time;
        } else if (time < T_2) {
            return MAX_VELOCITY;
        } else if (time <= T_3) {
            return MAX_VELOCITY - MAX_ACCELERATION * (time - T_2);
        } else {
            return 0;
        }
    } else {
        if (time < T_1) {
            return MAX_ACCELERATION * time;
        } else if (time <= T_3) {
            return V_PEAK - MAX_ACCELERATION * (time - T_2);
        } else {
            return 0;
        }
    }
}

double TrapezoidalProfile::getDuration() {
    return T_3;
}

namespace PathFollowing {
    void driveForward(double distance, ThreeWheelLocalizer localizer, 
    OdometryConstants odometry_constants, vex::motor_group left_motor_group,
    vex::motor_group right_motor_group) {
        TrapezoidalProfile profile = TrapezoidalProfile(distance, 1, 1);
        double duration = profile.getDuration();
        vex::timer timer = vex::timer();
        Pose prev_pose = localizer.getPoseEstimate();

        double elapsed_time = 0;
        timer.reset();
        double prev_time = timer.value();
        while (elapsed_time < duration) {
            double current_time = timer.value();
            double delta_time = current_time - prev_time;
            elapsed_time += delta_time;

            Pose current_pose = localizer.getPoseEstimate();
            double target_robot_vel = profile.calculateVelocity(elapsed_time);
            double target_wheel_vel = (
                target_robot_vel / odometry_constants.ODO_WHEEL_RADIUS);
            double target_wheel_vel_rad = target_wheel_vel * 180 * M_1_PI;
            left_motor_group.setVelocity(target_wheel_vel_rad, 
                vex::velocityUnits::dps);
            right_motor_group.setVelocity(target_wheel_vel_rad, 
                vex::velocityUnits::dps);
            
            prev_pose = current_pose;
            prev_time = current_time;
            /*
            double left_wheel_vel = left_motor_group.velocity(vex::velocityUnits::dps);
            double right_wheel_vel = right_motor_group.velocity(vex::velocityUnits::dps);
            */
        }
    }
}
