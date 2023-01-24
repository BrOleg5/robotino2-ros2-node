#include "robotino2/Robotino2Input.hpp"

Robotino2Input::Robotino2Input() : robotinoKinematics(0.04f, 0.130f, 16.f) {
    reset();
}

void Robotino2Input::reset() {
    // Set default PID controllers parameters
    kp.fill(255);
    ki.fill(255);
    kd.fill(255);
    velocitySetPoint.fill(0.f);
    resetPosition.fill(false);
    digitalOutput.fill(false);
    relay.fill(false);
    powerOutputControlPoint = 0;
    encoderInputResetPosition = false;
    shutdown = false;
    isDriveSystemControl = false;
    vx = 0.0f;
    vy = 0.0f;
    omega = 0.0f;
    maxVelocitySetPoint.fill(200.f);
}

bool Robotino2Input::setMotorVelocity(unsigned char motorNum, float velocity) {
    if(motorNum < 3) {
        if(std::abs(velocity) <= std::abs(maxVelocitySetPoint[motorNum])) {
            velocitySetPoint[motorNum] = velocity;
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool Robotino2Input::setMotorVelocities(const std::array<float, 3>& velocity) {
    if((std::abs(velocity[0]) <= std::abs(maxVelocitySetPoint[0])) &&
       (std::abs(velocity[1]) <= std::abs(maxVelocitySetPoint[1])) &&
       (std::abs(velocity[2]) <= std::abs(maxVelocitySetPoint[2]))) {
        velocitySetPoint = velocity;
        return true;
    }
    else {
        return false;
    }
}

bool Robotino2Input::setMotorVelocities(float omega1, float omega2, float omega3) {
    if((std::abs(omega1) <= std::abs(maxVelocitySetPoint[0])) &&
       (std::abs(omega2) <= std::abs(maxVelocitySetPoint[1])) &&
       (std::abs(omega3) <= std::abs(maxVelocitySetPoint[2]))) {
        velocitySetPoint[0] = omega1;
        velocitySetPoint[1] = omega2;
        velocitySetPoint[2] = omega3;
        return true;
    }
    else {
        return false;
    }
}

bool Robotino2Input::resetMotorPosition(unsigned char motorNum) {
    if(motorNum < 3) {
        resetPosition[motorNum] = true;
        return true;
    }
    else {
        return false;
    }
}

void Robotino2Input::resetMotorPositions() {
    resetPosition.fill(true);
}

bool Robotino2Input::setMotorPosition(unsigned char motorNum) {
    if(motorNum < 3) {
        resetPosition[motorNum] = false;
        return true;
    }
    else {
        return false;
    }
}

void Robotino2Input::setMotorPositions() {
    resetPosition.fill(false);
}

bool Robotino2Input::setMotorPID(unsigned char motorNum, unsigned char kp, unsigned char ki, unsigned char kd) {
    if(motorNum < 3) {
        this->kp[motorNum] = kp;
        this->ki[motorNum] = ki;
        this->kp[motorNum] = kp;
        return true;
    }
    else {
        return false;
    }
}

void Robotino2Input::setMotorPIDs(const std::array<unsigned char, 3>& kp, 
                                  const std::array<unsigned char, 3>& ki,
                                  const std::array<unsigned char, 3>& kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

bool Robotino2Input::setDigitalOutput(unsigned char outputNum) {
    if(outputNum < 8) {
        digitalOutput[outputNum] = true;
        return true;
    }
    else {
        return false;
    }
}

bool Robotino2Input::resetDigitalOutput(unsigned char outputNum) {
    if(outputNum < 8) {
        digitalOutput[outputNum] = false;
        return true;
    }
    else {
        return false;
    }
}

void Robotino2Input::setDigitalOutputs(const std::array<bool, 8>& outputState) {
    digitalOutput = outputState;
}

void Robotino2Input::setDigitalOutputs() {
    digitalOutput.fill(true);
}

void Robotino2Input::resetDigitalOutputs() {
    digitalOutput.fill(false);
}

bool Robotino2Input::setRelay(unsigned char relayNum) {
    if(relayNum < 2) {
        relay[relayNum] = true;
        return true;
    }
    else {
        return false;
    }
}
bool Robotino2Input::resetRelay(unsigned char relayNum) {
    if(relayNum < 2) {
        relay[relayNum] = false;
        return true;
    }
    else {
        return false;
    }
}

void Robotino2Input::setRelies(const std::array<bool, 2>& relayState) {
    relay = relayState;
}

void Robotino2Input::setRelies() {
    relay.fill(true);
}

void Robotino2Input::resetRelies() {
    relay.fill(false);
}

void Robotino2Input::setPowerOutputControlPoint(short powerOutput) {
    powerOutputControlPoint = powerOutput;
}

void Robotino2Input::resetEncoderInputPosition() {
    encoderInputResetPosition = true;
}

void Robotino2Input::setEncoderInputPosition() {
    encoderInputResetPosition = false;
}

void Robotino2Input::setShutdown() {
    shutdown = true;
}

void Robotino2Input::resetShutdown() {
    shutdown = false;
}

bool Robotino2Input::setRobotSpeed(float vx, float vy, float omega) {
    return setMotorVelocities(robotinoKinematics.inverse(vx, vy, omega));
}

void Robotino2Input::setMaxMotorVelocity(const std::array<float, 3>& max_vel) {
    maxVelocitySetPoint = max_vel;
}

void Robotino2Input::toTCPPayload(TransmitTCPPayload& buffer) const {
    //velocitySetPoint is in rpm, speed in inc/900ms
    float speed[3];
    for(unsigned char i = 0; i < 3; i++) {
        speed[i] = velocitySetPoint[i] * 2000.0f / 900.0f / 60.0f;
        if(speed[i] > 255.0f) {
            speed[i] = 255.0f;
        }
        else if(speed[i] < -255.0f) {
            speed[i] = -255.0f;
        }
    }

    buffer.shutdown = ( shutdown ? 1<<1 : 0 );

    /*This bit is ignored by Robotino's IO board with Atmel microcontroller
    The IO board with LPC microcontroller uses this bit to check, if old software (Robotino View <1.7) is connected.
    Old software does not set this bit. New software (Robotino View >1.7) sets this bit.
    If this bit is clear settings for kp, ki and kd are ignored.
    */
    buffer.shutdown |= ( 1<<3 );

    buffer.digitalOut1_4Relay1 = 1; //brake off
    buffer.digitalOut1_4Relay1 |= (digitalOutput[0] ? 1<<1 : 0);
    buffer.digitalOut1_4Relay1 |= (digitalOutput[1] ? 1<<2 : 0);
    buffer.digitalOut1_4Relay1 |= (digitalOutput[2] ? 1<<3 : 0);
    buffer.digitalOut1_4Relay1 |= (digitalOutput[3] ? 1<<4 : 0);
    buffer.digitalOut1_4Relay1 |= (relay[0] ? 1<<5 : 0);
    buffer.motor1Cmd = 0;
    if(speed[0] >= 0) {
        buffer.motor1Cmd = (1<<1);
    }
    if(resetPosition[0]) {
        buffer.motor1Cmd |= (1<<3);
    }
    buffer.motor1Speed = static_cast<unsigned char>(fabs(speed[0]));
    buffer.motor1Kp = kp[0];
    buffer.motor1Ki = ki[0];
    buffer.motor1Kd = kd[0];

    buffer.digitalOut5_8Relay2 = 1; //brake off
    buffer.digitalOut5_8Relay2 |= (digitalOutput[4] ? 1<<1 : 0);
    buffer.digitalOut5_8Relay2 |= (digitalOutput[5] ? 1<<2 : 0);
    buffer.digitalOut5_8Relay2 |= (digitalOutput[6] ? 1<<3 : 0);
    buffer.digitalOut5_8Relay2 |= (digitalOutput[7] ? 1<<4 : 0);
    buffer.digitalOut5_8Relay2 |= (relay[1] ? 1<<5 : 0);
    buffer.motor2Cmd = 0;
    if(speed[1] >= 0) {
        buffer.motor2Cmd = (1<<1);
    }
    if(resetPosition[1]) {
        buffer.motor2Cmd |= (1<<3);
    }
    buffer.motor2Speed = static_cast<unsigned char>(fabs(speed[1]));
    buffer.motor2Kp = kp[1];
    buffer.motor2Ki = ki[1];
    buffer.motor2Kd = kd[1];

    buffer.brakeOff1 = 1; //brake off
    buffer.motor3Cmd = 0;
    if(speed[2] >= 0) {
        buffer.motor3Cmd = (1<<1);
    }
    if(resetPosition[2]) {
        buffer.motor3Cmd |= (1<<3);
    }
    buffer.motor3Speed = static_cast<unsigned char>(fabs(speed[2]));
    buffer.motor3Kp = kp[2];
    buffer.motor3Ki = ki[2];
    buffer.motor3Kd = kd[2];

    buffer.brakeOff2 = 1; //brake off
    buffer.powerOutputEncoder[0] = 0;
    if(powerOutputControlPoint >= 0) {
        buffer.powerOutputEncoder[0] = (1<<1);
        buffer.powerOutputEncoder[1] = (powerOutputControlPoint & 0xFF);
    }
    else {
        buffer.powerOutputEncoder[1] = (-powerOutputControlPoint & 0xFF);
    }
    if(encoderInputResetPosition) {
        buffer.powerOutputEncoder[0] |= (1<<3);
    }
}