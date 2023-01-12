#ifndef ROBOTINO2NODE_HPP
#   define ROBOTINO2NODE_HPP 

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "robotino_interfaces/msg/motor_state.hpp"
#include "robotino_interfaces/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "robotino2/Robotino2.hpp"

using namespace std::chrono;

class Robotino2Node : public rclcpp::Node {
    public:
        Robotino2Node();

    private:
        void declare_node_parameters();
        void init_msgs();
        void cmd_vel_callback(const geometry_msgs::msg::Twist& msg);
        void mot_vel_callback(const robotino_interfaces::msg::MotorState& msg);
        void publish_all();
        void timer_callback();
        void reset_motor_positions_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                            const std::shared_ptr<std_srvs::srv::Empty::Response> response);

        Robotino2 robotino;

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription;
        rclcpp::Subscription<robotino_interfaces::msg::MotorState>::SharedPtr mot_vel_subscription;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_publisher;
        rclcpp::Publisher<robotino_interfaces::msg::MotorState>::SharedPtr motor_state_publisher;
        rclcpp::Publisher<robotino_interfaces::msg::Range>::SharedPtr distance_sensor_publisher;

        const milliseconds input_message_timeout;
        steady_clock::time_point last_input_message_time;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_motor_positions_service;

        robotino_interfaces::msg::MotorState motor_state_msg;
        robotino_interfaces::msg::Range range_msg;
};

#endif