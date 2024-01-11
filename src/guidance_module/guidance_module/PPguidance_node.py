import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from shipsim_msgs_module.msg import PPguid
from rclpy.executors import SingleThreadedExecutor


class PPguidanceNode(Node):
    def __init__(self, ship_number):
        super().__init__('guidance')
        self.ship_number=ship_number
        self.now_wp_id=0
