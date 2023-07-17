import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from shipsim_msgs_module.msg import Control

class SensorNode(Node):
    def __init__(self):
        pass

    def listener_callback(self):
        self.get_logger().info()

    def sender_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node=SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdow()

if __name__=="__main__":
    main()