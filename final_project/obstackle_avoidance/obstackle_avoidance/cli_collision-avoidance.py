import click
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
import queue
import time
from math import inf

@click.group()
def robot_controller_cli():
    """CLI tool for Robot Controller"""
    pass


@robot_controller_cli.command()
@click.option('--kP', type=float, default=0.2, help='Proportional gain')
@click.option('--kI', type=float, default=0.01, help='Integral gain')
@click.option('--kD', type=float, default=0.2, help='Derivative gain')
@click.option('--kS', type=int, default=10, help='Saturation constant')
def pid_controller(kP, kI, kD, kS):
    """Run PID Controller with specified gains"""
    pid = PIDController(kP, kI, kD, kS)
    # Perform actions or print information based on PID Controller initialization


@robot_controller_cli.command()
@click.option('--scan-topic', default='/scan', help='LaserScan topic name')
@click.option('--cmd-topic', default='/cmd_vel', help='Command topic name')
@click.option('--delay', type=float, default=4.0, help='Time delay (s)')
def start_controller(scan_topic, cmd_topic, delay):
    """Start the Robot Controller Node"""
    rclpy.init()

    class RobotControllerNode(Node):
        def __init__(self):
            super().__init__('robot_controller')

    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    robot_controller_cli()
