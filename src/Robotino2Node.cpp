#include "robotino2/Robotino2Node.hpp"

void Robotino2Node::cmd_vel_callback(const geometry_msgs::msg::Twist& msg) {
    // RCLCPP_INFO(this->get_logger(), "Receive set speed: %f, %f, %f", msg.linear.x, msg.linear.y, msg.angular.z);
    if(!robotino.input.setRobotSpeed(static_cast<float>(msg.linear.x), static_cast<float>(msg.linear.y), static_cast<float>(msg.angular.z))) {
        RCLCPP_WARN(this->get_logger(), "Set speed is very high!");
    }
    last_input_message_time = steady_clock::now();
}

void Robotino2Node::mot_vel_callback(const robotino_interfaces::msg::MotorVelocities& msg) {
    // RCLCPP_INFO(this->get_logger(), "Receive set motor velocities: %f, %f, %f", msg.vel[0], msg.vel[1], msg.vel[2]);
    if(!robotino.input.setMotorVelocities(msg.vel[0], msg.vel[1], msg.vel[2])) {
        RCLCPP_WARN(this->get_logger(), "Set velocity is very high!");
    }
    last_input_message_time = steady_clock::now();
}

void Robotino2Node::publish_all() {
    std_msgs::msg::Bool bumper_msg;
    bumper_msg.data = robotino.output.getBumperState();
    bumper_publisher_->publish(bumper_msg);

    robotino_interfaces::msg::MotorPositions mot_pos_msg;
    mot_pos_msg.pos = robotino.output.getMotorPositions();
    motor_positions_publisher_->publish(mot_pos_msg);

    robotino_interfaces::msg::MotorVelocities mot_vel_msg;
    mot_vel_msg.vel = robotino.output.getMotorVelocities();
    motor_velocities_publisher_->publish(mot_vel_msg);

    robotino_interfaces::msg::MotorCurrents mot_cur_msg;
    mot_cur_msg.cur = robotino.output.getMotorCurrents();
    motor_currents_publisher_->publish(mot_cur_msg);

    robotino_interfaces::msg::DistanceSensorVoltages dist_sens_msg;
    dist_sens_msg.vol = robotino.output.getDistanceSensorVoltages();
    distance_sensor_publisher_->publish(dist_sens_msg);
}

void Robotino2Node::timer_callback() {
    if(duration_cast<milliseconds>(steady_clock::now() - last_input_message_time) > input_message_timeout) {
        robotino.input.setRobotSpeed(0.f, 0.f, 0.f);
        robotino.input.setMotorVelocities(0.f, 0.f, 0.f);
    }
    robotino.communicate_once();
    this->publish_all();
    robotino.input.setMotorPositions();
}

void Robotino2Node::reset_motor_positions_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                   const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    robotino.input.resetMotorPositions();
    RCLCPP_INFO(this->get_logger(), "Motor positions reset.");
}