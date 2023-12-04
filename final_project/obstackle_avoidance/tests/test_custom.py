from emergency_braking.main import RobotController

def test_obstacle_detection():
    # Instantiate RobotController node
    robot_controller = RobotController()

    # Create a LaserScan message object with obstacle detected at a certain distance
    obstacle_detected_distance = 0.5  # Set an obstacle detected at 0.5 meters
    laser_scan_msg = LaserScan()
    laser_scan_msg.ranges = [obstacle_detected_distance]  # Set the ranges attribute with obstacle distance

    # Call the laser scan callback function manually
    robot_controller.robot_laserscan_callback(laser_scan_msg)

    # Run the robot_controller_callback to check emergency braking behavior
    robot_controller.robot_controller_callback()

    # Validate that the robot performs emergency braking if an obstacle is detected within threshold
    assert robot_controller.ctrl_msg.linear.x == 0.0  # Check if linear velocity is set to 0.0 for emergency braking
    assert robot_controller.ctrl_msg.angular.z == 0.0  # Check if angular velocity is set to 0.0 for emergency braking
