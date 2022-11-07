#ifndef ROBOTINOKINEMATICS_HPP
#   define ROBOTINOKINEMATICS_HPP

#include <array>

#include "robotino2/utils.hpp"

class RobotinoKinematics {
public:
        RobotinoKinematics(float wheelRadius, float robotRadius, float gearRatio) : wheelRadius(wheelRadius),
                                                                                    robotRadius(robotRadius), 
                                                                                    gearRatio(gearRatio) {};

        std::array<float, 3> inverse(float vx, float vy, float omega);
        std::array<float, 3> direct(const std::array<float, 3>& wheelVelocity);
        std::array<float, 3> direct(float omega1, float omega2, float omega3);

        inline float getWheelRadius() {
            return wheelRadius;
        }

        inline float getRobotRadius() {
            return robotRadius;
        }
        inline float getGearRatio() {
            return gearRatio;
        }

    private:
        float wheelRadius;
        float robotRadius;
        float gearRatio;
        const std::array<float, 3> wheelAngle = {PI / 3.f, PI, 5.f * PI / 3.f};
};

#endif