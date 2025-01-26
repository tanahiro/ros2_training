
#ifndef TRN_HPP_
#define TRN_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "random_numbers/random_numbers.h"

#include "trn_interfaces/srv/command.hpp"

class TrnNode : public rclcpp::Node {
public:
    TrnNode();

    void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<trn_interfaces::srv::Command::Request> request,
        const std::shared_ptr<trn_interfaces::srv::Command::Response> response);

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<trn_interfaces::srv::Command>::SharedPtr srv_cmd_;

    void timer_callback();
};

#endif