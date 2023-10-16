#include "rclcpp/rclcpp.hpp"
#include "warehouse_msgs/action/warehouse_task.hpp"

using namespace std::placeholders;

class WarehouseActionServer : public rclcpp::Node
{
public:
    WarehouseActionServer()
        : Node("warehouse_action_server"), server_(this, "warehouse_task", std::bind(&WarehouseActionServer::execute, this, _1))
    {
    }

private:
    rclcpp::Server<warehouse_msgs::action::WarehouseTask>::SharedPtr server_;

    // Callback function to execute the requested task
    void execute(const std::shared_ptr<warehouse_msgs::action::WarehouseTask::GoalHandle> goal_handle)
    {
        rclcpp::Rate loop_rate(1);
        bool success = true;
        std::string result = "Task completed successfully";

        // Implement task execution logic here.
        // This is where you would add the code to control the robot's actions.

        if (rclcpp::ok())
        {
            goal_handle->succeed(std::make_shared<warehouse_msgs::action::WarehouseTask::Result>(warehouse_msgs::action::WarehouseTask::Result{success, result}));
            RCLCPP_INFO(this->get_logger(), "Task completed");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WarehouseActionServer>());
    rclcpp::shutdown();
    return 0;
}