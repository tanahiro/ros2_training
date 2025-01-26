#include "trn/trn.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;


TrnNode::TrnNode() : Node("trn") {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);

    timer_ = this->create_wall_timer(
        200ms,
        std::bind(&TrnNode::timer_callback, this)
    );

    srv_cmd_ = this->create_service<trn_interfaces::srv::Command>(
        "trn/command",
        std::bind(&TrnNode::handle_service, this, _1, _2, _3)
    );
}

void TrnNode::handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<trn_interfaces::srv::Command::Request> request,
    const std::shared_ptr<trn_interfaces::srv::Command::Response> response)
{
    (void)request_header;
    RCLCPP_INFO(get_logger(), "request: %s", request->cmd.c_str());
    response->state = "ok";
}

void TrnNode::timer_callback() {
    auto rnd = random_numbers::RandomNumberGenerator();

    double position = rnd.uniformReal(90, 100);
    double velocity = rnd.uniformReal(-1, 1);
    double effort = rnd.uniformReal(-1, 1);

    auto state = sensor_msgs::msg::JointState();
    state.header.stamp = this->get_clock()->now();
    state.name = {"gripper"};
    state.position = {position};
    state.velocity = {velocity};
    state.effort = {effort};

    RCLCPP_INFO(this->get_logger(), "publish (%f %f %f)", position, velocity, effort);
    publisher_->publish(state);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TrnNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}