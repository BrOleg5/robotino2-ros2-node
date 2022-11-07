#ifndef ROBOTINO2NODE_HPP
#   define ROBOTINO2NODE_HPP 

#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robotino_interfaces/msg/distance_sensor_voltages.hpp"
#include "robotino_interfaces/msg/motor_currents.hpp"
#include "robotino_interfaces/msg/motor_positions.hpp"
#include "robotino_interfaces/msg/motor_velocities.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "robotino2/Robotino2.hpp"

using namespace std::chrono;
using std::placeholders::_1;

class Robotino2Node : public rclcpp::Node {
    public:
        Robotino2Node() : Node("robotino2"), input_message_timeout(200) {
            this->declare_parameter("ip", "172.26.1.0");
            this->declare_parameter("port", 80);
            this->declare_parameter("sample_time", 20);

            std::string address = this->get_parameter("ip").get_parameter_value().get<std::string>();
            int port = static_cast<int>(this->get_parameter("port").get_parameter_value().get<long long>());
            robotino.connect(address, port);
            RCLCPP_INFO(this->get_logger(), "Connect to Robotino2 %s:%d", address.c_str(), port);

            std::string node_name = this->get_name();
            cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(node_name + "/cmd_vel", 10, 
                                    std::bind(&Robotino2Node::cmd_vel_callback, this, _1));
            mot_vel_subscription_ = this->create_subscription<robotino_interfaces::msg::MotorVelocities>(node_name + "/cmd_mot_vel", 10,
                                    std::bind(&Robotino2Node::mot_vel_callback, this, _1));

            bumper_publisher_ = this->create_publisher<std_msgs::msg::Bool>(node_name + "/bumper", 10);
            motor_positions_publisher_ = this->create_publisher<robotino_interfaces::msg::MotorPositions>(node_name + "/mot_pos", 10);
            motor_velocities_publisher_ = this->create_publisher<robotino_interfaces::msg::MotorVelocities>(node_name + "/mot_vel", 10);
            motor_currents_publisher_ = this->create_publisher<robotino_interfaces::msg::MotorCurrents>(node_name + "/mot_cur", 10);
            distance_sensor_publisher_ = this->create_publisher<robotino_interfaces::msg::DistanceSensorVoltages>(node_name + "/dist_sens", 10);

            milliseconds sample_time = milliseconds(this->get_parameter("sample_time").get_parameter_value().get<long long>());
            timer_ = this->create_wall_timer(sample_time, std::bind(&Robotino2Node::timer_callback, this));
        }

    private:
        void cmd_vel_callback(const geometry_msgs::msg::Twist& msg);
        void mot_vel_callback(const robotino_interfaces::msg::MotorVelocities& msg);
        void publish_all();
        void timer_callback();

        Robotino2 robotino;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
        rclcpp::Subscription<robotino_interfaces::msg::MotorVelocities>::SharedPtr mot_vel_subscription_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_publisher_;
        rclcpp::Publisher<robotino_interfaces::msg::MotorPositions>::SharedPtr motor_positions_publisher_;
        rclcpp::Publisher<robotino_interfaces::msg::MotorVelocities>::SharedPtr motor_velocities_publisher_;
        rclcpp::Publisher<robotino_interfaces::msg::MotorCurrents>::SharedPtr motor_currents_publisher_;
        rclcpp::Publisher<robotino_interfaces::msg::DistanceSensorVoltages>::SharedPtr distance_sensor_publisher_;

        const milliseconds input_message_timeout;
        steady_clock::time_point last_input_message_time;
};

#endif