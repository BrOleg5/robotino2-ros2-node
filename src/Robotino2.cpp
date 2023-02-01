// Copyright 2022 BrOleg5

#include "robotino2/Robotino2.hpp"

Robotino2::Robotino2(): input(), output(), socket(ios), startPayload(5) {
    transmitTCPPtr = reinterpret_cast<unsigned char*>(&transmitTCPPayload);
    startMessage = buffer + startPayload;
}

Robotino2::~Robotino2() {
    socket.close();
}

bool Robotino2::connect(const std::string& address, int port) {
    endpoint = boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(address), port);
    socket.connect(endpoint, error);
    if(error) {
        std::cerr << "Error: " << error.what() << '\n';
        return false;
    } else {
        return true;
    }
}

bool Robotino2::connect(const char* address, int port) {
    endpoint = boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(address), port);
    socket.connect(endpoint, error);
    if(error) {
        std::cerr << "Error: " << error.what() << '\n';
        return false;
    } else {
        return true;
    }
}

bool Robotino2::start_communication() {
    input.toTCPPayload(transmitTCPPayload);
    socket.write_some(boost::asio::buffer(transmitTCPPtr, sizeof(transmitTCPPayload)), error);
    if(error) {
        std::cerr << "Error: " << error.what() << '\n';
        return false;
    }
    socket.read_some(boost::asio::buffer(buffer, bufSize), error);
    if(error) {
        std::cerr << "Error: " << error.what() << '\n';
        return false;
    }
    socket.read_some(boost::asio::buffer(buffer, bufSize), error);
    if(error) {
        std::cerr << "Error: " << error.what() << '\n';
        return false;
    }
    const unsigned char start_message[8] = { 0x02, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    socket.write_some(boost::asio::buffer(start_message, sizeof(start_message)), error);
    if(error) {
        std::cerr << "Error: " << error.what() << '\n';
        return false;
    }
    socket.read_some(boost::asio::buffer(buffer, bufSize), error);
    if(error) {
        std::cerr << "Error: " << error.what() << '\n';
        return false;
    }
    return true;
}

bool Robotino2::communicate_once() {
    input.toTCPPayload(transmitTCPPayload);
    socket.write_some(boost::asio::buffer(transmitTCPPtr, sizeof(transmitTCPPayload)), error);
    if(error) {
        std::cerr << "Error: " << error.what() << '\n';
        return false;
    }
    socket.read_some(boost::asio::buffer(buffer, bufSize), error);
    if(error) {
        std::cerr << "Error: " << error.what() << '\n';
        return false;
    }
    if(!output.fromTCPPayload(startMessage)) {
        std::cerr << "Error parse TCP payload.\n";
    }
    return true;
}
