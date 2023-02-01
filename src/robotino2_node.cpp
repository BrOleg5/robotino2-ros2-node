// Copyright 2022 BrOleg5

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "robotino2/Robotino2Node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robotino2Node>());
    rclcpp::shutdown();

    return 0;
}
