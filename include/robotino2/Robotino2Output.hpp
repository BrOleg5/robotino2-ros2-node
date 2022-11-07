#ifndef ROBOTINO2OUTPUT_HPP
#   define ROBOTINO2OUTPUT_HPP

#include <iostream>
#include <array>

#include "robotino2/utils.hpp"

class Robotino2Output {
public:
    Robotino2Output();

    void reset();

    bool fromTCPPayload(const unsigned char* data);

    std::array<int, 3> getMotorPositions();
    std::array<float, 3> getMotorVelocities();
    std::array<float, 3> getMotorCurrents();
    std::array<bool, 8> getDigitalInputs();
    std::array<float, 8> getAnalogInputs();
    std::array<float, 9> getDistanceSensorVoltages();

    inline int getMotorPosition(unsigned char motorNum) {
        return motorPosition[motorNum];
    }

    inline float getMotorVelocity(unsigned char motorNum) {
        return rpm2rads(motorVelocity[motorNum]);
    }

    inline float getMotorCurrent(unsigned char motorNum) {
        return motorCurrent[motorNum];
    }

    inline bool getDigitalInput(unsigned char inputNum) {
        return dIn[inputNum];
    }

    float getAnalogInput(unsigned char inputNum) {
        return aIn[inputNum];
    }

    inline float getDistanceSensorVoltage(unsigned char sensorNum) {
        return distanceSensor[sensorNum];
    }

    inline bool getBumperState() {
        return bumper;
    }

    inline float getPowerSupplyCurrent() {
        return current;
    }

    inline float getPowerSupplyVoltage() {
        return voltage;
    }

    inline unsigned int getFirmwareVersion() {
        return firmwareVersion;
    }

    inline unsigned int getSequenceNumber() {
        return sequenceNumber;
    }

private:
    float powerOutputCurrent;
    unsigned short powerOutputRawCurrent;

    int encoderInputPosition;
    int encoderInputVelocity;

    std::array<float, 3> motorVelocity;
    std::array<int, 3> motorPosition;
    std::array<float, 3> motorCurrent;
    std::array<unsigned short, 3> rawMotorCurrent;

    //digital inputs
    std::array<bool, 8> dIn;

    //analog inputs
    std::array<float, 8> aIn;

    std::array<float, 9> distanceSensor;

    bool bumper;

    float current;
    float voltage;

    unsigned int firmwareVersion;

    unsigned int sequenceNumber;

    //EA09 firmware version
    //If this is {0,0,0} we have a EA05
    std::array<unsigned char, 3> firmware_version;
};

#endif