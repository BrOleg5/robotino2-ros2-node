// Copyright 2022 BrOleg5

#ifndef ROBOTINO2__ROBOTINOKINEMATICS_HPP_
#define ROBOTINO2__ROBOTINOKINEMATICS_HPP_

#include <array>

#include "robotino2/utils.hpp"

class RobotinoKinematics {
public:
        RobotinoKinematics(float wheelRadius,
                           float robotRadius,
                           float gearRatio) : wheelRadius(wheelRadius),
                                              robotRadius(robotRadius),
                                              gearRatio(gearRatio) {}

        inline float getWheelRadius();
        inline float getRobotRadius();
        inline float getGearRatio();

        std::array<float, 3> motor_inverse(float vx, float vy, float omega);
        std::array<float, 3> inverse(float vx, float vy, float omega);
        std::array<float, 3> direct(const std::array<float, 3>& wheelVelocity);
        std::array<float, 3> direct(float omega1, float omega2, float omega3);

    private:
        const float wheelRadius;
        const float robotRadius;
        const float gearRatio;
        const std::array<float, 3> wheelAngle = {PI / 3.f, PI, 5.f * PI / 3.f};
};

inline float RobotinoKinematics::getWheelRadius() {
    return wheelRadius;
}

inline float RobotinoKinematics::getRobotRadius() {
    return robotRadius;
}
inline float RobotinoKinematics::getGearRatio() {
    return gearRatio;
}

#endif  // ROBOTINO2__ROBOTINOKINEMATICS_HPP_
