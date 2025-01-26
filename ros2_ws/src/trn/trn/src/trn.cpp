#include "trn/trn.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

TrnNode::TrnNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("trn", options)
{
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("trn/joint_state", 10);

    timer_ = this->create_wall_timer(
        500ms,
        std::bind(&TrnNode::timer_callback, this)
    );

    srv_cmd_ = this->create_service<trn_interfaces::srv::Command>(
        "trn/command",
        std::bind(&TrnNode::handle_service, this, _1, _2, _3)
    );

    action_server_cmd_ = rclcpp_action::create_server<trn_interfaces::action::Command>(
        this,
        "trn/action_cmd",
        std::bind(&TrnNode::action_cmd_goal, this, _1, _2),
        std::bind(&TrnNode::action_cmd_cancel, this, _1),
        std::bind(&TrnNode::action_cmd_accepted, this, _1));
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

rclcpp_action::GoalResponse TrnNode::action_cmd_goal(
    const rclcpp_action::GoalUUID &uuid ,
    std::shared_ptr<const trn_interfaces::action::Command::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal request %s", goal->cmd.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrnNode::action_cmd_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<trn_interfaces::action::Command>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TrnNode::action_cmd_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<trn_interfaces::action::Command>> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TrnNode::action_cmd_execute, this, _1), goal_handle}.detach();
}

void TrnNode::action_cmd_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<trn_interfaces::action::Command>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "start execution");
    rclcpp::Rate loop_rate(1);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<trn_interfaces::action::Command::Feedback>();

    auto result = std::make_shared<trn_interfaces::action::Command::Result>();

    for (int i = 0; i < 10 && rclcpp::ok(); i++) {
        RCLCPP_INFO(this->get_logger(), "executing %d", i);

        loop_rate.sleep();
    }

    if (rclcpp::ok()) {
        result->result = "completed";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "action completed");
    }
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