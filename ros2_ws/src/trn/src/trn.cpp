
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "random_numbers/random_numbers.h"

using namespace std::chrono_literals;

class TrnNode : public rclcpp::Node
{
public:
    TrnNode() : Node("trn") {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&TrnNode::timer_callback, this)
        );
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
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
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrnNode>());
  rclcpp::shutdown();
  return 0;
}