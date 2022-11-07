#include "robotino2/Robotino2.hpp"

bool Robotino2::connect(const std::string& address, int port) {
    endpoint = boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(address), port);
    socket.connect(endpoint);
    if(error) {
        std::cout << "Error: " << error.what() << '\n';
        return false;
    }
    else {
        return true;
    }
}

bool Robotino2::connect(const char* address, int port) {
    endpoint = boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(address), port);
    socket.connect(endpoint);
    if(error) {
        std::cout << "Error: " << error.what() << '\n';
        return false;
    }
    else {
        return true;
    }
}

bool Robotino2::start_communication() {
    const unsigned char start_message[8] = { 0x02, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    socket.write_some(boost::asio::buffer(start_message, sizeof(start_message)), error);
    if(error) {
        std::cout << "Error: " << error.what() << '\n';
        return false;
    }
    socket.read_some(boost::asio::buffer(buffer, bufSize - startPayload), error);
    if(error) {
        std::cout << "Error: " << error.what() << '\n';
        return false;
    }
    return true;
}

bool Robotino2::communicate_once() {
    input.toTCPPayload(transmitTCPPayload);
    socket.write_some(boost::asio::buffer(transmitTCPPointer, sizeof(transmitTCPPayload)), error);
    if(error) {
        std::cout << "Error: " << error.what() << '\n';
        return false;
    }
    std::this_thread::sleep_for(10ms);
    socket.read_some(boost::asio::buffer(buffer, bufSize - startPayload), error);
    if(error) {
        std::cout << "Error: " << error.what() << '\n';
        return false;
    }
    output.fromTCPPayload(startMessage);
    return true;
}