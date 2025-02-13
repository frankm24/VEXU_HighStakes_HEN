#define _USE_MATH_DEFINES

#include "odometry.h"
#include <cmath>

ThreeWheelLocalizer::ThreeWheelLocalizer(Pose initial_pose, DriveConstants odometry_constants, 
    vex::encoder& left_encoder, vex::encoder& right_encoder, vex::encoder& offset_encoder) : 
    DIST_TRACK_WIDTH(odometry_constants.DIST_TRACK_WIDTH),
    DIST_FRONT_OFFSET(odometry_constants.DIST_FRONT_OFFSET),
    ODO_WHEEL_CIRCUMFERENCE(2 * M_PI * odometry_constants.ODO_WHEEL_RADIUS),
    left_encoder(left_encoder),
    right_encoder(right_encoder),
    offset_encoder(offset_encoder),
    current_pose(initial_pose),
    previous_pose(initial_pose)
    {
    }

void ThreeWheelLocalizer::update(double delta_time) {
    double left_encoder_position = left_encoder.position(vex::turns);
    double right_encoder_position = right_encoder.position(vex::turns);
    double offset_encoder_position = offset_encoder.position(vex::turns);

    double delta_left_encoder = (left_encoder_position - prev_left_encoder_position) * ODO_WHEEL_CIRCUMFERENCE;
    double delta_right_encoder = (right_encoder_position - prev_right_encoder_position) * ODO_WHEEL_CIRCUMFERENCE;
    double delta_offset_encoder = (offset_encoder_position - prev_offset_encoder_position) * ODO_WHEEL_CIRCUMFERENCE;

    double delta_x = (delta_left_encoder + delta_right_encoder) / 2;
    double delta_theta = (delta_right_encoder - delta_left_encoder) / DIST_TRACK_WIDTH;
    double delta_y = delta_offset_encoder - DIST_FRONT_OFFSET * (delta_right_encoder - delta_left_encoder) / DIST_TRACK_WIDTH;

    double current_x = previous_pose.x + delta_x*std::cos(previous_pose.heading) - delta_y*std::sin(previous_pose.heading);
    double current_y = previous_pose.y + delta_x*std::sin(previous_pose.heading) + delta_y*std::cos(previous_pose.heading);
    double current_heading = previous_pose.heading + delta_theta;

    current_pose = {current_x, current_y, current_heading};
    current_velocity = (current_pose - previous_pose) / delta_time;
    previous_pose = {current_x, current_y, current_heading};

    prev_left_encoder_position = left_encoder_position;
    prev_right_encoder_position = right_encoder_position;
    prev_offset_encoder_position = offset_encoder_position;
}

Pose ThreeWheelLocalizer::getPoseEstimate() {
    return current_pose;
}
Pose ThreeWheelLocalizer::getVelocityEstimate() {
    return current_velocity;
}
