import rclpy
from rclpy.action import ActionServer
from fleet_management_interfaces.action import FleetManagement
from std_msgs.msg import String
from std_srvs.srv import Empty
from rclpy.node import Node

class FleetManagementServer(Node):

    def __init__(self):
        super().__init__('fleet_management_server')
        self.action_server = ActionServer(
            self,
            FleetManagement,
            'fleet_management',
            self.fleet_management_callback)

    def fleet_management_callback(self, goal_handle):
        self.get_logger().info('Received fleet management request')
        fleet_size = goal_handle.request.fleet_size
        self.get_logger().info(f'Received request for a fleet of size {fleet_size}')

        # Perform fleet management logic (e.g., allocation and routing)
        vehicle_routes = self.calculate_vehicle_routes(fleet_size)

        if not vehicle_routes:
            goal_handle.succeed()
            result = FleetManagement.Result(vehicle_routes=[])
            goal_handle.set_succeeded(result)
        else:
            for route in vehicle_routes:
                self.get_logger().info(f'Calculated route: {route}')
                goal_handle.publish_feedback(FleetManagement.Feedback(completion_percentage=0.0))

            goal_handle.succeed()
            result = FleetManagement.Result(vehicle_routes=vehicle_routes)
            goal_handle.set_succeeded(result)

    def calculate_vehicle_routes(self, fleet_size):
        # Implement your fleet management logic here
        # This is a mock implementation that generates dummy routes
        if fleet_size < 1:
            return []
        routes = [f'Route for Vehicle {i+1}' for i in range(fleet_size)]
        return routes

def main(args=None):
    rclpy.init(args=args)
    fleet_management_server = FleetManagementServer()
    try:
        rclpy.spin(fleet_management_server)
    except KeyboardInterrupt:
        pass
    fleet_management_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
