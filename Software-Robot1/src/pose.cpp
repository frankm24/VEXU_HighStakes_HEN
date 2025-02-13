#include "pose.h"
#include <cmath>

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
double Pose::distance() {
    return sqrt(
        pow(x, 2) + pow(y, 2)
    );
}