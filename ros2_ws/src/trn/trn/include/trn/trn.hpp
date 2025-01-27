
#ifndef TRN_HPP_
#define TRN_HPP_

#include <cinttypes>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "random_numbers/random_numbers.h"

#include "trn_interfaces/srv/command.hpp"
#include "trn_interfaces/action/command.hpp"

class TrnNode : public rclcpp::Node {
public:
    explicit TrnNode(const rclcpp::NodeOptions &options);

    void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<trn_interfaces::srv::Command::Request> request,
        const std::shared_ptr<trn_interfaces::srv::Command::Response> response);

    rclcpp_action::GoalResponse action_cmd_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const trn_interfaces::action::Command::Goal>);
    rclcpp_action::CancelResponse action_cmd_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<trn_interfaces::action::Command>>);
    void action_cmd_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<trn_interfaces::action::Command>>);

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<trn_interfaces::srv::Command>::SharedPtr srv_cmd_;
    rclcpp_action::Server<trn_interfaces::action::Command>::SharedPtr action_server_cmd_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_handler_;

    void timer_callback();
    void action_cmd_execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<trn_interfaces::action::Command>>);
    void param_callback(const rclcpp:: Parameter&);
};

#endif