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

    inline int getMotorPosition(unsigned char motorNum);
    inline float getMotorVelocity(unsigned char motorNum);
    inline float getMotorCurrent(unsigned char motorNum);
    inline bool getDigitalInput(unsigned char inputNum);
    inline float getAnalogInput(unsigned char inputNum);
    inline float getDistanceSensorVoltage(unsigned char sensorNum);
    inline bool getBumperState();
    inline float getPowerSupplyCurrent();
    inline float getPowerSupplyVoltage();
    inline unsigned int getFirmwareVersion();
    inline unsigned int getSequenceNumber();
    std::array<int, 3> getMotorPositions();
    std::array<float, 3> getMotorVelocities();
    std::array<float, 3> getMotorCurrents();
    std::array<bool, 8> getDigitalInputs();
    std::array<float, 8> getAnalogInputs();
    std::array<float, 9> getDistanceSensorVoltages();

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

inline int Robotino2Output::getMotorPosition(unsigned char motorNum) {
    return motorPosition[motorNum];
}

inline float Robotino2Output::getMotorVelocity(unsigned char motorNum) {
    return rpm2rads(motorVelocity[motorNum]);
}

inline float Robotino2Output::getMotorCurrent(unsigned char motorNum) {
    return motorCurrent[motorNum];
}

inline bool Robotino2Output::getDigitalInput(unsigned char inputNum) {
    return dIn[inputNum];
}

inline float Robotino2Output::getAnalogInput(unsigned char inputNum) {
    return aIn[inputNum];
}

inline float Robotino2Output::getDistanceSensorVoltage(unsigned char sensorNum) {
    return distanceSensor[sensorNum];
}

inline bool Robotino2Output::getBumperState() {
    return bumper;
}

inline float Robotino2Output::getPowerSupplyCurrent() {
    return current;
}

inline float Robotino2Output::getPowerSupplyVoltage() {
    return voltage;
}

inline unsigned int Robotino2Output::getFirmwareVersion() {
    return firmwareVersion;
}

inline unsigned int Robotino2Output::getSequenceNumber() {
    return sequenceNumber;
}

#endif