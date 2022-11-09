#ifndef ROBOTINO2_HPP
#   define ROBOTINO2_HPP

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include <boost/asio.hpp>

#include "robotino2/Robotino2Input.hpp"
#include "robotino2/Robotino2Output.hpp"

using namespace std::chrono_literals;

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
        unsigned char* transmitTCPPointer;

        static const unsigned int bufSize = 1024;
        static const unsigned int startPayload = 5;
        unsigned char buffer[bufSize];
        unsigned char* startMessage;
    
};

#endif