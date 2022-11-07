#include "robotino2/RobotinoKinematics.hpp"

std::array<float, 3> RobotinoKinematics::inverse(float vx, float vy, float omega) {
    std::array<float, 3> wheelVelocity;
    for (unsigned char i = 0; i < 3; i++) {
        wheelVelocity[i] = gearRatio * (-vx * sin(wheelAngle[i]) + vy * cos(wheelAngle[i]) + omega * robotRadius) / wheelRadius;   
    }
    return wheelVelocity;
}

std::array<float, 3> RobotinoKinematics::direct(const std::array<float, 3>& wheelVelocity) {
    std::array<float, 3> robotSpeed;
    robotSpeed[0] = -2.f * wheelRadius / 3.f * (sin(wheelAngle[0]) * wheelVelocity[0] +
                                                sin(wheelAngle[1]) * wheelVelocity[1] + 
                                                sin(wheelAngle[2]) * wheelVelocity[2]);
    robotSpeed[1] = 2.f * wheelRadius / 3.f * (cos(wheelAngle[0]) * wheelVelocity[0] +
                                                cos(wheelAngle[1]) * wheelVelocity[1] + 
                                                cos(wheelAngle[2]) * wheelVelocity[2]);
    robotSpeed[1] = 2.f * wheelRadius / 3.f / robotRadius * (wheelVelocity[0] + wheelVelocity[1] + wheelVelocity[2]);
    return robotSpeed;
}

std::array<float, 3> RobotinoKinematics::direct(float omega1, float omega2, float omega3) {
    std::array<float, 3> robotSpeed;
    robotSpeed[0] = -2.f * wheelRadius / 3.f * (sin(wheelAngle[0]) * omega1 +
                                                sin(wheelAngle[1]) * omega2 + 
                                                sin(wheelAngle[2]) * omega3);
    robotSpeed[1] = 2.f * wheelRadius / 3.f * (cos(wheelAngle[0]) * omega1 +
                                                cos(wheelAngle[1]) * omega2 + 
                                                cos(wheelAngle[2]) * omega3);
    robotSpeed[1] = 2.f * wheelRadius / 3.f / robotRadius * (omega1 + omega2 + omega3);
    return robotSpeed;
}