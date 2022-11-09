#include "robotino2/Robotino2Node.hpp"

Robotino2Node::Robotino2Node() : Node("robotino2"), input_message_timeout(200) {
    declare_node_parameters();

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

void Robotino2Node::declare_node_parameters() {
    rcl_interfaces::msg::ParameterDescriptor ip_param_desc{};
    ip_param_desc.description = "IP-address of Festo Robotino 2. Eg.: 172.26.1.0";
    this->declare_parameter("ip", "172.26.1.0", ip_param_desc);
    rcl_interfaces::msg::ParameterDescriptor port_param_desc{};
    port_param_desc.description = "Festo Robotino 2 communication port. Default value: 80.";
    this->declare_parameter("port", 80, port_param_desc);
    rcl_interfaces::msg::ParameterDescriptor sample_period_param_desc{};
    sample_period_param_desc.description = "Sample period for send and receive Robotino control and sensors data. Sample period in ms.";
    this->declare_parameter("sample_period", 20, sample_period_param_desc);
}

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