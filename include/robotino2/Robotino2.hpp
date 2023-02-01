// Copyright 2022 BrOleg5

#ifndef ROBOTINO2__ROBOTINO2_HPP_
#define ROBOTINO2__ROBOTINO2_HPP_

#include <iostream>
#include <vector>
#include <string>

#include <boost/asio.hpp>

#include "robotino2/Robotino2Input.hpp"
#include "robotino2/Robotino2Output.hpp"

class Robotino2 {
    public:
        Robotino2();
        ~Robotino2();

        bool connect(const std::string& address, int port);
        bool connect(const char* address, int port);
        bool start_communication();

        bool communicate_once();

        Robotino2Input input;
        Robotino2Output output;

    private:
        boost::asio::io_service ios;
        boost::asio::ip::tcp::endpoint endpoint;
        boost::asio::ip::tcp::socket socket;
        boost::system::error_code error;

        TransmitTCPPayload transmitTCPPayload;
        unsigned char* transmitTCPPtr;

        static const unsigned int bufSize = 1024;
        const unsigned int startPayload;
        unsigned char buffer[bufSize];
        unsigned char* startMessage;
};

#endif  // ROBOTINO2__ROBOTINO2_HPP_
