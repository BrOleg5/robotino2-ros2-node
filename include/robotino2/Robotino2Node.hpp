#ifndef ROBOTINO2NODE_HPP
#   define ROBOTINO2NODE_HPP 

#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "robotino_interfaces/msg/distance_sensor_voltages.hpp"
#include "robotino_interfaces/msg/motor_currents.hpp"
#include "robotino_interfaces/msg/motor_positions.hpp"
#include "robotino_interfaces/msg/motor_velocities.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "robotino2/Robotino2.hpp"

using namespace std::chrono;

class Robotino2Node : public rclcpp::Node {
    public:
        Robotino2Node() : Node("robotino2"), input_message_timeout(200) {
            rcl_interfaces::msg::ParameterDescriptor ip_param_desc{};
            ip_param_desc.description = "IP-address of Festo Robotino 2. Eg.: 172.26.1.0";
            this->declare_parameter("ip", "172.26.1.0", ip_param_desc);
            rcl_interfaces::msg::ParameterDescriptor port_param_desc{};
            port_param_desc.description = "Festo Robotino 2 communication port. Default value: 80.";
            this->declare_parameter("port", 80, port_param_desc);
            rcl_interfaces::msg::ParameterDescriptor sample_period_param_desc{};
            sample_period_param_desc.description = "Sample period for send and receive Robotino control and sensors data. Sample period in ms.";
            this->declare_parameter("sample_period", 20, sample_period_param_desc);

            std::string address = this->get_parameter("ip").get_parameter_value().get<std::string>();
            int port = static_cast<int>(this->get_parameter("port").get_parameter_value().get<long long>());
            robotino.connect(address, port);
            RCLCPP_INFO(this->get_logger(), "Connect to Robotino2 %s:%d", address.c_str(), port);

            std::string node_name = this->get_name();
            cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(node_name + "/cmd_vel", 10, 
                                    std::bind(&Robotino2Node::cmd_vel_callback, this, std::placeholders::_1));
            mot_vel_subscription_ = this->create_subscription<robotino_interfaces::msg::MotorVelocities>(node_name + "/cmd_mot_vel", 10,
                                    std::bind(&Robotino2Node::mot_vel_callback, this, std::placeholders::_1));

            bumper_publisher_ = this->create_publisher<std_msgs::msg::Bool>(node_name + "/bumper", 10);
            motor_positions_publisher_ = this->create_publisher<robotino_interfaces::msg::MotorPositions>(node_name + "/mot_pos", 10);
            motor_velocities_publisher_ = this->create_publisher<robotino_interfaces::msg::MotorVelocities>(node_name + "/mot_vel", 10);
            motor_currents_publisher_ = this->create_publisher<robotino_interfaces::msg::MotorCurrents>(node_name + "/mot_cur", 10);
            distance_sensor_publisher_ = this->create_publisher<robotino_interfaces::msg::DistanceSensorVoltages>(node_name + "/dist_sens", 10);

            reset_motor_positions_service_ = this->create_service<std_srvs::srv::Empty>(node_name + "/reset_pos", 
                                             std::bind(&Robotino2Node::reset_motor_positions_callback, this, 
                                             std::placeholders::_1, std::placeholders::_2));

            milliseconds sample_period = milliseconds(this->get_parameter("sample_period").get_parameter_value().get<long long>());
            timer_ = this->create_wall_timer(sample_period, std::bind(&Robotino2Node::timer_callback, this));
        }

    private:
        void cmd_vel_callback(const geometry_msgs::msg::Twist& msg);
        void mot_vel_callback(const robotino_interfaces::msg::MotorVelocities& msg);
        void publish_all();
        void timer_callback();
        void reset_motor_positions_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                            const std::shared_ptr<std_srvs::srv::Empty::Response> response);

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

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_motor_positions_service_;
};

#endif