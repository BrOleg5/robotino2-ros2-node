// Copyright 2022 BrOleg5

#ifndef ROBOTINO2__TCPPAYLOAD_HPP_
#define ROBOTINO2__TCPPAYLOAD_HPP_

#pragma pack(push, 1)
struct TransmitTCPPayload {
    unsigned char prefix[12] = {0x00, 0x4c, 0x00, 0x00, 0x00, 0x90,
                                0x1f, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char startBytes[3] = {'R', 'E', 'C'};
    unsigned char shutdown;
    unsigned char digitalOut1_4Relay1;
    unsigned char motor1Cmd;
    unsigned char motor1Speed;
    unsigned char dummyBytes1[4] = { 0 };
    unsigned char motor1Kp;
    unsigned char motor1Ki;
    unsigned char motor1Kd;
    unsigned char digitalOut5_8Relay2;
    unsigned char motor2Cmd;
    unsigned char motor2Speed;
    unsigned char dummyBytes2[4] = { 0 };
    unsigned char motor2Kp;
    unsigned char motor2Ki;
    unsigned char motor2Kd;
    unsigned char brakeOff1;
    unsigned char motor3Cmd;
    unsigned char motor3Speed;
    unsigned char dummyBytes3[4] = { 0 };
    unsigned char motor3Kp;
    unsigned char motor3Ki;
    unsigned char motor3Kd;
    unsigned char brakeOff2;
    unsigned char powerOutputEncoder[2];
    unsigned char dummyBytes4[7] = { 0 };
    unsigned char endBytes[3] = {'r', 'e', 'c'};
    unsigned char postfix[15] = { 0x00 };
    unsigned char postpostfix[5] = {0x80, 0x3f, 0x00, 0x00, 0x00};
};
#pragma pack(pop)

#endif  // ROBOTINO2__TCPPAYLOAD_HPP_
