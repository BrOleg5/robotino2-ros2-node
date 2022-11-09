#ifndef ROBOTINO2INPUT_HPP
#   define ROBOTINO2INPUT_HPP

#include <cmath>
#include <array>

#include "robotino2/TCPPayload.hpp"
#include "robotino2/RobotinoKinematics.hpp"
#include "robotino2/utils.hpp"

class Robotino2Input {
public:
    Robotino2Input() : robotinoKinematics(0.04f, 0.130f, 16.f) {
        reset();
    }

    void reset();

    bool setMotorVelocity(unsigned char motorNum, float velocity);
    bool setMotorVelocities(const std::array<float, 3>& velocity);
    bool setMotorVelocities(float omega1, float omega2, float omega3);
    bool resetMotorPosition(unsigned char motorNum);
    void resetMotorPositions();
    bool setMotorPosition(unsigned char motorNum);
    void setMotorPositions();
    bool setMotorPID(unsigned char motorNum, unsigned char kp, unsigned char ki, unsigned char kd);
    void setMotorPIDs(const std::array<unsigned char, 3>& kp, const std::array<unsigned char, 3>& ki, 
                      const std::array<unsigned char, 3>& kd);
    bool setDigitalOutput(unsigned char outputNum);
    bool resetDigitalOutput(unsigned char outputNum);
    void setDigitalOutputs(const std::array<bool, 8>& outputState);
    void setDigitalOutputs();
    void resetDigitalOutputs();
    bool setRelay(unsigned char relayNum);
    bool resetRelay(unsigned char relayNum);
    void setRelies(const std::array<bool, 2>& relayState);
    void setRelies();
    void resetRelies();
    void setPowerOutputControlPoint(short powerOutput);
    void resetEncoderInputPosition(); // ?
    void setEncoderInputPosition(); // ?
    void setShutdown();
    void resetShutdown();
    bool setRobotSpeed(float vx, float vy, float omega);
    void setMaxMotorVelocity(const std::array<float, 3>& max_vel);

    void toTCPPayload(TransmitTCPPayload& buffer) const;

private:
    std::array<float, 3> velocitySetPointRPM;

    std::array<bool, 3> resetPosition;

    // Coefficients of motor PID controllers
    std::array<unsigned char, 3> kp;
    std::array<unsigned char, 3> ki;
    std::array<unsigned char, 3> kd;

    std::array<bool, 8> digitalOutput;

    std::array<bool, 2> relay;

    /**Range [-255;255] */
    short powerOutputControlPoint;

    bool encoderInputResetPosition;

    bool shutdown;

    bool isDriveSystemControl;
    float vx;
    float vy;
    float omega;

    RobotinoKinematics robotinoKinematics;

    std::array<float, 3> maxVelocitySetPoint;
};

#endif