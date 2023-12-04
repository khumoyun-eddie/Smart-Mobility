# Author: Khumoyun Rakhmonberdiev 12194949
#!/usr/bin/env python3

# ROS2 module imports
import rclpy # ROS2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # Node class for Python nodes
from geometry_msgs.msg import Twist # Twist (linear and angular velocities) message class
from sensor_msgs.msg import LaserScan # LaserScan (LIDAR range measurements) message class
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
from rclpy.duration import Duration # Time duration class

# Python mudule imports
from math import pi # Common mathematical constant

# Node class
class RobotController(Node):

    #######################
    '''Class constructor'''
    #######################

    def __init__(self):
        # Information and debugging
        info = '\nMake the robot stop if obstacles are detected within a threshold distance.\n'
        print(info)
        # ROS2 infrastructure
        super().__init__('robot_controller') # Create a node with name 'robot_controller'
        qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=10 # Queue size/depth of 10 (only honored if the “history” policy was set to “keep last”)
        )
        self.robot_scan_sub = self.create_subscription(LaserScan, '/scan', self.robot_laserscan_callback, qos_profile) # Subscriber which will subscribe to LaserScan message on the topic '/scan' adhering to 'qos_profile' QoS profile
        self.robot_scan_sub # Prevent unused variable warning
        self.robot_ctrl_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile) # Publisher which will publish Twist message to the topic '/cmd_vel' adhering to 'qos_profile' QoS profile
        timer_period = 0.001 # Node execution time period (seconds)
        self.timer = self.create_timer(timer_period, self.robot_controller_callback) # Define timer to execute 'robot_controller_callback()' every 'timer_period' seconds
        self.laserscan = [] # Initialize variable to capture the laserscan
        self.ctrl_msg = Twist() # Robot control commands (twist)
        self.start_time = self.get_clock().now() # Record current time in seconds

    ########################
    '''Callback functions'''
    ########################

    def robot_laserscan_callback(self, msg):
        self.laserscan = msg.ranges # Capture most recent laserscan

    def robot_controller_callback(self):
        THRESH = 1.0 # Distance threshold to perform emergency braking (m)
        LIN_VEL = 1.0 # Linear velocity (m/s)
        ANG_VEL = 0.0 # Angular velocity (rad/s)
        DELAY = 4.0 # Time delay (s)
        if self.get_clock().now() - self.start_time > Duration(seconds=DELAY):
            if self.laserscan[0] >= THRESH: # Let robot continue its motion
                self.ctrl_msg.linear.x = LIN_VEL # Set linear velocity
                self.ctrl_msg.angular.z = ANG_VEL # Set angular velocity
                self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
                print('Obstacle detected at {} m'.format(self.laserscan[0]))
                #print('Robot moving with {} m/s and {} rad/s'.format(LIN_VEL, ANG_VEL))
            else:
                self.ctrl_msg.linear.x = 0.0 # Set linear velocity
                self.ctrl_msg.angular.z = 0.0 # Set angular velocity
                self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
                print('Emergency braking performed!')
                #quit() # Quit execution
        else:
            print('Initializing...')

def main(args=None):
    rclpy.init(args=args) # Start ROS2 communications
    node = RobotController() # Create node
    rclpy.spin(node) # Execute node
    node.destroy_node() # Destroy node explicitly (optional - otherwise it will be done automatically when garbage collector destroys the node object)
    rclpy.shutdown() # Shutdown ROS2 communications

if __name__ == "__main__":
    main()
