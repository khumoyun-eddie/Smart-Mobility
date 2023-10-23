import rclpy
from rclpy.action import ActionClient
from fleet_management_interfaces.action import FleetManagement
from std_msgs.msg import String
from std_srvs.srv import Empty
from rclpy.node import Node

class FleetManagementClient(Node):

    def __init__(self):
        super().__init__('fleet_management_client')
        self.action_client = ActionClient(self, FleetManagement, 'fleet_management')
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.send_request()

    def send_request(self):
        fleet_size = int(input('Enter fleet size: '))
        goal_msg = FleetManagement.Goal(fleet_size=fleet_size)

        self.get_logger().info(f'Sending request for a fleet of size {fleet_size}')
        self.action_client.send_goal(goal_msg, feedback_callback=self.feedback_callback)

        # Wait for the result
        result = self.action_client.wait_for_result()
        if result:
            routes = result.result.vehicle_routes
            if not routes:
                self.get_logger().info('No routes calculated for the fleet.')
            else:
                self.get_logger().info('Received the following routes:')
                for i, route in enumerate(routes):
                    self.get_logger().info(f'Route for Vehicle {i + 1}: {route}')
        else:
            self.get_logger().info('Action server did not respond within the specified timeout.')

    def feedback_callback(self, feedback_msg):
        completion_percentage = feedback_msg.feedback.completion_percentage
        self.get_logger().info(f'Completion Percentage: {completion_percentage * 100}%')

def main(args=None):
    rclpy.init(args=args)
    fleet_management_client = FleetManagementClient()
    rclpy.spin(fleet_management_client)
    fleet_management_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
