#include "robotino2/Robotino2Output.hpp"

Robotino2Output::Robotino2Output() {
    reset();   
}

void Robotino2Output::reset() {
    motorPosition.fill(0);
    motorVelocity.fill(0.f);
    motorCurrent.fill(0.f);
    rawMotorCurrent.fill(0);
    dIn.fill(false);
    aIn.fill(0.f);
    distanceSensor.fill(0.f);
    bumper = false;
    current = 0.0f;
    voltage = 0.0f;
    powerOutputCurrent = 0.0f;
    powerOutputRawCurrent = 0;
    encoderInputPosition = 0;
    encoderInputVelocity = 0;
    firmwareVersion = 0;
    sequenceNumber = 0;
    firmware_version.fill(0);
}

bool Robotino2Output::fromTCPPayload(const unsigned char* data) {
    unsigned short distance[9] = {0};
    unsigned short ad[8] = {0};
    short avint[3] = {0};
    unsigned short currentint;
    unsigned short voltageint;
    unsigned char* uint8p;

    if( *(data)     != 'R' ||
        *(data+1)   != 'E' ||
        *(data+2)   != 'C' ||
        *(data+98)  != 'r' ||
        *(data+99)  != 'e' ||
        *(data+100) != 'c' ) 
    {
        return false;
    }

    //master start at byte 3
    currentint = (*(data+3) << 2);
    voltageint = (*(data+5) << 2);

    firmware_version[0] = *(data+7);
    firmware_version[1] = *(data+8);
    firmware_version[2] = *(data+9);

    currentint |= (*(data+11) & 0x3);
    voltageint |= (( *(data+11) >> 4 ) & 0x3);

    //slave 0 start at byte 14
    rawMotorCurrent[0] = (*(data+14) << 2);
    distance[5] = (*(data+15) << 2);
    distance[6] = (*(data+16) << 2);
    ad[0] = (*(data+17) << 2);
    ad[1] = (*(data+18) << 2);
    ad[2] = (*(data+19) << 2);
    ad[3] = (*(data+20) << 2);
    distance[7] = (*(data+21) << 2);

    rawMotorCurrent[0] |= (*(data+22) & 0x3);
    distance[5] |= ((*(data+22) >> 2 ) & 0x3);
    distance[6] |= ((*(data+22) >> 4 ) & 0x3);
    ad[0] |= ((*(data+22) >> 6 ) & 0x3);

    ad[1] |= ((*(data+23) ) & 0x3);
    ad[2] |= ((*(data+23) >> 2 ) & 0x3);
    ad[3] |= ((*(data+23) >> 4 ) & 0x3);
    distance[7] |= ((*(data+23) >> 6 ) & 0x3);

    firmwareVersion = 0;
    firmwareVersion |= ((*(data+24) & 1<<4 ) ? 1<<3 : 0);
    firmwareVersion |= ((*(data+24) & 1<<5 ) ? 1<<2 : 0);
    firmwareVersion |= ((*(data+24) & 1<<6 ) ? 1<<1 : 0);
    firmwareVersion |= ((*(data+24) & 1<<7 ) ? 1 : 0);

    avint[0] = *(data+25);
    if( (*(data+24) & 0x1 ) == 0) {
        avint[0] = -avint[0];
    }

    uint8p = reinterpret_cast<unsigned char*>(&motorPosition[0]);
    *uint8p++ = *(data+26);
    *uint8p++ = *(data+27);
    *uint8p++ = *(data+28);
    *uint8p = *(data+29);
    motorPosition[0] = -motorPosition[0];

    //digital input
    dIn[0] = ((*(data+30) & 1 ) > 0);
    dIn[1] = ((*(data+30) & 1<<1 ) > 0);
    dIn[2] = ((*(data+30) & 1<<2 ) > 0);
    dIn[3] = ((*(data+30) & 1<<3 ) > 0);
    bumper = (( *(data+30) & 1<<4 ) > 0);

    sequenceNumber = *reinterpret_cast<const unsigned int*>(data+31);

    //slave 1 start at byte 35
    rawMotorCurrent[1] = (*(data+35) << 2);
    distance[4] = (*(data+36) << 2);
    distance[3] = (*(data+37) << 2);
    ad[4] = (*(data+38) << 2);
    ad[5] = (*(data+39) << 2);
    ad[6] = (*(data+40) << 2);
    ad[7] = (*(data+41) << 2);
    distance[2] = (*(data+42) << 2);

    rawMotorCurrent[1] |= (*(data+43) & 0x3);
    distance[4] |= ((*(data+43) >> 2 ) & 0x3);
    distance[3] |= ((*(data+43) >> 4 ) & 0x3);
    ad[4] |= ((*(data+43) >> 6 ) & 0x3);

    ad[5] |= ((*(data+44) ) & 0x3);
    ad[6] |= ((*(data+44) >> 2 ) & 0x3);
    ad[7] |= ((*(data+44) >> 4 ) & 0x3);
    distance[2] |= ((*(data+44) >> 6) & 0x3);

    avint[1] = *(data+46);
    if((*(data+45) & 0x1) == 0 ) {
        avint[1] = -avint[1];
    }

    uint8p = reinterpret_cast<unsigned char*>(&motorPosition[1]);
    *uint8p++ = *(data+47);
    *uint8p++ = *(data+48);
    *uint8p++ = *(data+49);
    *uint8p = *(data+50);
    motorPosition[1] = -motorPosition[1];

    dIn[4] = ((*(data+51) & 1 ) > 0);
    dIn[5] = ((*(data+51) & 1<<1 ) > 0);
    dIn[6] = ((*(data+51) & 1<<2 ) > 0);
    dIn[7] = ((*(data+51) & 1<<3 ) > 0);

    //slave 2 start at byte 56
    rawMotorCurrent[2] = (*(data+56) << 2);
    distance[1] = (*(data+57) << 2);

    //AD2
    distance[0] = (*(data+58) << 2);

    distance[8] = (*(data+63) << 2);


    rawMotorCurrent[2] |= (*(data+64) & 0x3);
    distance[1] |= ( (*(data+64) >> 2 ) & 0x3);
    distance[0] |= ( (*(data+64) >> 4 ) & 0x3);

    distance[8] |= ((*(data+65) >> 6 ) & 0x3);

    avint[2] = *(data+67);
    if((*(data+66) & 0x1) == 0) {
        avint[2] = -avint[2];
    }

    uint8p = reinterpret_cast<unsigned char*>(&motorPosition[2]);
    *uint8p++ = *(data+68);
    *uint8p++ = *(data+69);
    *uint8p++ = *(data+70);
    *uint8p = *(data+71);
    motorPosition[2] = -motorPosition[2];

    //slave 3 start at byte 77
    powerOutputRawCurrent = (*(data+77) << 2);
    powerOutputRawCurrent |= (*(data+85) & 0x3);

    powerOutputCurrent = static_cast<float>(powerOutputRawCurrent) / 156.0f;


    encoderInputVelocity = *(data+88);
    if((*(data+87) & 0x1) == 0) {
        encoderInputVelocity = -encoderInputVelocity;
    }

    uint8p = reinterpret_cast<unsigned char*>(&encoderInputPosition);
    *uint8p++ = *(data+89);
    *uint8p++ = *(data+90);
    *uint8p++ = *(data+91);
    *uint8p = *(data+92);
    encoderInputPosition = -encoderInputPosition;

    for(unsigned int i = 0; i < 9; i++) {
        distanceSensor[i] = 2.55f / 1024.0f * distance[i];
    }

    for(unsigned int i = 0; i < 8; i++) {
        aIn[i] = 10.0f / 1024.0f * ad[i];
    }

    for(unsigned int i = 0; i < 3; i++) {
        motorVelocity[i] = 27.0f * avint[i]; // 900 * 60 / 2000 = 27
        motorCurrent[i] = static_cast<float>( rawMotorCurrent[i] ) / 156.0f;
    }

    voltage = static_cast<float>(voltageint) / 36.10f;
    current = static_cast<float>(currentint) / 61.44f;

    return true;
}

std::array<int, 3> Robotino2Output::getMotorPositions() {
    std::array<int, 3> actualPosition;
    for (unsigned char i = 0; i < 3; i++) {
        actualPosition[i] = motorPosition[i];
    }
    return actualPosition;
}

std::array<float, 3> Robotino2Output::getMotorVelocities() {
    std::array<float, 3> actualVelocity;
    for (unsigned char i = 0; i < 3; i++) {
        actualVelocity[i] = rpm2rads(motorVelocity[i]);
    }
    return actualVelocity;
}

std::array<float, 3> Robotino2Output::getMotorCurrents() {
    std::array<float, 3> actualCurrent;
    for (unsigned char i = 0; i < 3; i++) {
        actualCurrent[i] = motorCurrent[i];
    }
    return actualCurrent;
}

std::array<bool, 8> Robotino2Output::getDigitalInputs() {
    std::array<bool, 8> digitalInput;
    for (unsigned char i = 0; i < 8; i++) {
        digitalInput[i] = dIn[i];
    }
    return digitalInput;
}

std::array<float, 8> Robotino2Output::getAnalogInputs() {
    std::array<float, 8> analogInput;
    for (unsigned char i = 0; i < 8; i++) {
        analogInput[i] = aIn[i];
    }
    return analogInput;
}

std::array<float, 9> Robotino2Output::getDistanceSensorVoltages() {
    std::array<float, 9> sensorVoltage;
    for (unsigned char i = 0; i < 9; i++) {
        sensorVoltage[i] = distanceSensor[i];
    }
    return sensorVoltage;
}