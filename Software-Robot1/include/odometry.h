#ifndef ODOMETRY_H
#define ODOMETRY_H
//#pragma once

#include "vex.h"
#include "pose.h"

struct OdometryConstants {
    const double DIST_TRACK_WIDTH;
    const double DIST_FRONT_OFFSET;
    const double ODO_WHEEL_RADIUS;
};

class ThreeWheelLocalizer {
    private:
        const double DIST_TRACK_WIDTH;
        const double DIST_FRONT_OFFSET;
        const double ODO_WHEEL_CIRCUMFERENCE;
        
        vex::encoder left_encoder, right_encoder, offset_encoder;
 
        Pose current_pose;
        Pose previous_pose;
        Pose current_velocity;

        double prev_left_encoder_position = 0 ;
        double prev_right_encoder_position = 0;
        double prev_offset_encoder_position = 0;


    public:
        ThreeWheelLocalizer(Pose initial_pose, 
        OdometryConstants odometry_constants, vex::encoder& left_encoder, 
        vex::encoder& right_encoder, vex::encoder& offset_encoder);
        void update(double delta_time);
        Pose getPoseEstimate();
        Pose getVelocityEstimate();
};

#endif
