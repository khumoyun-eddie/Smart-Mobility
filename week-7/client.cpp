#include "rclcpp/rclcpp.hpp"
#include "warehouse_msgs/action/warehouse_task.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("warehouse_action_client");
    auto client = rclcpp::Node::make_shared("warehouse_task");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}