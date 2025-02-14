/*
#include "path_planning.h"
#include "cmath"
#include "controllers.h"

// NOT TESTED

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
    DriveConstants drive_constants, vex::motor_group left_motor_group,
    vex::motor_group right_motor_group) {
        TrapezoidalProfile profile = TrapezoidalProfile(distance, 0.5, 2);
        PIDController position_pid = PIDController(0.5, 0, 0.05);
        PIDController heading_pid = PIDController(0.1, 0, 0.01);

        double duration = profile.getDuration();
        vex::timer timer = vex::timer();

        Pose start_pose = localizer.getPoseEstimate();
        Pose prev_pose = start_pose;
        double target_distance = 0;
        double target_heading = start_pose.heading;
        double elapsed_time = 0;
        timer.reset();
        double prev_time = timer.value();

        while (elapsed_time < duration) {
            double current_time = timer.value();
            double delta_time = current_time - prev_time;
            elapsed_time += delta_time;

            localizer.update(delta_time);
            Pose current_pose = localizer.getPoseEstimate();

            double actual_distance = (current_pose - start_pose).distance();
            double target_robot_vel = profile.calculateVelocity(elapsed_time);
            target_distance += target_robot_vel * delta_time;

            double position_error = target_distance - actual_distance;
            double vel_adj = position_pid.calculate(position_error, delta_time);

            double heading_error = target_heading - current_pose.heading;
            double heading_adj = heading_pid.calculate(heading_error, delta_time);

            target_robot_vel += vel_adj;
            double target_wheel_vel = target_robot_vel / drive_constants.ODO_WHEEL_RADIUS;
            double target_wheel_vel_deg = target_wheel_vel * 180 * M_1_PI;
            
            double target_left_speed = target_wheel_vel_deg - heading_adj;
            double target_right_speed = target_wheel_vel_deg + heading_adj;

            left_motor_group.setVelocity(target_left_speed, 
                vex::velocityUnits::dps);
            right_motor_group.setVelocity(target_right_speed, 
                vex::velocityUnits::dps);
            
            prev_pose = current_pose;
            prev_time = current_time;
        }
        left_motor_group.setVelocity(0, vex::velocityUnits::pct);
        right_motor_group.setVelocity(0, vex::velocityUnits::pct);
    }
    void rotateDegrees(double degrees, ThreeWheelLocalizer localizer,
    DriveConstants odometry_constants, vex::motor_group left_motor_group,
    vex::motor_group right_motor_group) {
        TrapezoidalProfile profile = TrapezoidalProfile(fabs(degrees), 180, 360); 
        PIDController heading_pid = PIDController(0.5, 0, 0.05);
        
        double duration = profile.getDuration();
        vex::timer timer = vex::timer();
        Pose prev_pose = localizer.getPoseEstimate();

        double target_heading = prev_pose.heading;
        double current_heading = prev_pose.heading;

        double elapsed_time = 0;
        timer.reset();
        double prev_time = timer.value();
        while (elapsed_time < duration) {
            double current_time = timer.value();
            double delta_time = current_time - prev_time;
            elapsed_time += delta_time;
            localizer.update(delta_time);

            Pose current_pose = localizer.getPoseEstimate();
            double target_angular_vel = profile.calculateVelocity(elapsed_time);
            if (degrees < 0) {
                target_angular_vel *= -1;
            }
            double target_left_wheel_vel = (-target_angular_vel * odometry_constants.DIST_TRACK_WIDTH / 2) / odometry_constants.ODO_WHEEL_RADIUS;
            double target_right_wheel_vel = (target_angular_vel * odometry_constants.DIST_TRACK_WIDTH / 2) / odometry_constants.ODO_WHEEL_RADIUS;

            double target_left_wheel_vel_deg = target_left_wheel_vel * 180 * M_1_PI;
            double target_right_wheel_vel_deg = target_left_wheel_vel * 180 * M_1_PI;
            
            double left_error = target_left_wheel_vel_deg - left_motor_group.velocity(vex::velocityUnits::dps);
            double right_error = target_right_wheel_vel_deg - right_motor_group.velocity(vex::velocityUnits::dps);

            double left_adj = left_pid.calculate(left_error, delta_time);
            double right_adj = right_pid.calculate(right_error, delta_time);

            left_motor_group.setVelocity(target_left_wheel_vel_deg, 
                vex::velocityUnits::dps);
            right_motor_group.setVelocity(target_right_wheel_vel_deg, 
                vex::velocityUnits::dps);
            
            prev_pose = current_pose;
            prev_time = current_time;
        }
        left_motor_group.setVelocity(0, vex::velocityUnits::pct);
        right_motor_group.setVelocity(0, vex::velocityUnits::pct);
    }
}
*/