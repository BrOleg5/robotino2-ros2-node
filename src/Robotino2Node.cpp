#include "robotino2/Robotino2Node.hpp"

Robotino2Node::Robotino2Node() : Node("robotino2"), input_message_timeout(200) {
    declare_node_parameters();

    std::string address = this->get_parameter("ip").get_parameter_value().get<std::string>();
    int port = static_cast<int>(this->get_parameter("port").get_parameter_value().get<long long>());
    robotino.connect(address, port);
    RCLCPP_INFO(this->get_logger(), "Connect to Robotino2 %s:%d", address.c_str(), port);

    std::string node_name = this->get_name();
    cmd_vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>("/" + node_name + "/cmd_vel", 10, 
                            std::bind(&Robotino2Node::cmd_vel_callback, this, std::placeholders::_1));
    mot_vel_subscription = this->create_subscription<robotino_interfaces::msg::MotorState>("/" + node_name + "/cmd_mot_vel", 10,
                            std::bind(&Robotino2Node::mot_vel_callback, this, std::placeholders::_1));

    bumper_publisher = this->create_publisher<std_msgs::msg::Bool>("/" + node_name + "/bumper", 10);
    std::string topic_name = "/" + node_name + "/motor_state";
    motor_state_publisher = this->create_publisher<robotino_interfaces::msg::MotorState>(topic_name, 10);
    
    topic_name = "/" + node_name + "/ir";
    distance_sensor_publisher = this->create_publisher<robotino_interfaces::msg::Range>(topic_name, 10);

    init_msgs();

    reset_motor_positions_service = this->create_service<std_srvs::srv::Empty>("/" + node_name + "/reset_pos", 
                                        std::bind(&Robotino2Node::reset_motor_positions_callback, this, 
                                        std::placeholders::_1, std::placeholders::_2));

    milliseconds sample_period = milliseconds(this->get_parameter("sample_period").get_parameter_value().get<long long>());
    timer = this->create_wall_timer(sample_period, std::bind(&Robotino2Node::timer_callback, this));
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

void Robotino2Node::init_msgs() {
    for (unsigned char i = 0; i < 3; i++) {
        motor_state_msg.name[i] = "motor_" + std::to_string(i+1);
    }

    range_msg.radiation_type = range_msg.INFRARED;
    range_msg.field_of_view = 0.7f; // Will correct in the future
    range_msg.min_range = 0.04f;
    range_msg.max_range = 0.30f;
}

void Robotino2Node::cmd_vel_callback(const geometry_msgs::msg::Twist& msg) {
    // RCLCPP_INFO(this->get_logger(), "Receive set speed: %f, %f, %f", msg.linear.x, msg.linear.y, msg.angular.z);
    if(!robotino.input.setRobotSpeed(static_cast<float>(msg.linear.x), static_cast<float>(msg.linear.y), static_cast<float>(msg.angular.z))) {
        RCLCPP_WARN(this->get_logger(), "Set speed is very high!");
    }
    last_input_message_time = steady_clock::now();
}

void Robotino2Node::mot_vel_callback(const robotino_interfaces::msg::MotorState& msg) {
    // RCLCPP_INFO(this->get_logger(), "Receive set motor velocities: %f, %f, %f", msg.vel[0], msg.vel[1], msg.vel[2]);
    if(!robotino.input.setMotorVelocities(msg.velocity)) {
        RCLCPP_WARN(this->get_logger(), "Set velocity is very high!");
    }
    last_input_message_time = steady_clock::now();
}

void Robotino2Node::publish_all() {
    builtin_interfaces::msg::Time stamp = this->get_clock()->now();
    std_msgs::msg::Bool bumper_msg;
    bumper_msg.data = robotino.output.getBumperState();
    bumper_publisher->publish(bumper_msg);

    motor_state_msg.header.stamp = stamp;
    motor_state_msg.position = robotino.output.getMotorPositions();
    motor_state_msg.velocity = robotino.output.getMotorVelocities();
    motor_state_msg.current = robotino.output.getMotorCurrents();
    motor_state_publisher->publish(motor_state_msg);

    range_msg.header.stamp = stamp;
    range_msg.range = robotino.output.getDistanceSensorVoltages();
    distance_sensor_publisher->publish(range_msg);
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