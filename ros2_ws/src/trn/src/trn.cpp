
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class TrnNode : public rclcpp::Node
{
public:
    TrnNode() : Node("trn") {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrnNode>());
  rclcpp::shutdown();
  return 0;
}