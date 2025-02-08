#include "pose.h"

Pose Pose::operator+(const Pose& other) const {
    return {x + other.x, y + other.y, heading + other.heading};
}
Pose Pose::operator-(const Pose& other) const {
    return {x - other.x, y - other.y, heading - other.heading};
}
Pose Pose::operator*(double scalar) const {
    return {x * scalar, y * scalar, heading * scalar};
}
Pose Pose::operator/(double scalar) const {
    return {x / scalar, y / scalar, heading / scalar};
}