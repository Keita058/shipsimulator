import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from shipsim_msgs_module.msg import Control

class SensorNode(Node):
    def __init__(self):
        super().__init__("sensor", namespace="ship1")

        self.declare_parameter("subscribe_address1", "/ship1/obs_pose")
        subscribe_address1=(self.get_parameter("subscribe_address1").get_parameter_value().string_value)
        self.subscription = self.create_subscription(
            Twist, subscribe_address1, self.listener_callback1, 10
        )

        self.declare_parameter("subscribe_address2", "/ship1/obs_vel")
        subscribe_address2=(self.get_parameter("subscribe_address2").get_parameter_value().string_value)
        self.subscription = self.create_subscription(
            Twist, subscribe_address2, self.listener_callback2, 10
        )

    def listener_callback1(self,msg):
        self.get_logger().info('Subscribe: x="%ss", y="%s", psi="%s"'%(msg.linear.x, msg.linear.y, msg.angular.z))

    def listener_callback2(self,msg):
        self.get_logger().info('Subscribe: u="%ss", v="%s", r="%s"'%(msg.linear.x, msg.linear.y, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node=SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdow()

if __name__=="__main__":
    main()