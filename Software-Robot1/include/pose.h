#ifndef POSE_H
#define POSE_H

struct Pose {
    public:
        double x;
        double y;
        double heading;
        Pose operator-(const Pose& other) const;
        Pose operator+(const Pose& other) const;
        Pose operator*(double scalar) const;
        Pose operator/(double scalar) const;
        double distance();
};

#endif