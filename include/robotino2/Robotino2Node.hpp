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
        Robotino2Node();

    private:
        void declare_node_parameters();
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