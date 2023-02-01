// Copyright 2022 BrOleg5

#ifndef ROBOTINO2__ROBOTINO2NODE_HPP_
#define ROBOTINO2__ROBOTINO2NODE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "extra_robot_interfaces/msg/motor_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "extra_robot_interfaces/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "robotino2/Robotino2.hpp"

using extra_robot_interfaces::msg::MotorState;
using extra_robot_interfaces::msg::Range;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

class Robotino2Node : public rclcpp::Node {
    public:
        Robotino2Node();

    private:
        void declare_node_parameters();
        void init_msgs();
        void cmd_vel_callback(const geometry_msgs::msg::Twist& msg);
        void mot_vel_callback(const MotorState& msg);
        void publish_all();
        void timer_callback();
        void reset_motor_positions_callback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            const std::shared_ptr<std_srvs::srv::Empty::Response> response
        );

        Robotino2 robotino;

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription;
        rclcpp::Subscription<MotorState>::SharedPtr mot_vel_subscription;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_publisher;
        rclcpp::Publisher<MotorState>::SharedPtr motor_state_publisher;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
        rclcpp::Publisher<Range>::SharedPtr distance_sensor_publisher;

        const milliseconds input_message_timeout;
        steady_clock::time_point last_input_message_time;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_motor_positions_service;

        sensor_msgs::msg::JointState joint_state_msg;
        MotorState motor_state_msg;
        Range range_msg;
};

#endif  // ROBOTINO2__ROBOTINO2NODE_HPP_
