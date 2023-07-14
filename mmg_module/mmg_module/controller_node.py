#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from shipsim_msgs_module.msg import Control
from sensor_msgs.msg import Joy

class MmgControllerNode(Node):
    """ControllerNode."""
    n_p=0.0
    rudder_angle_degree=0.0

    def __init__(self):
        """init."""
        super().__init__("controller", namespace="ship1")

        self.declare_parameter("subscribe_address","/joy")
        subscribe_address=(self.get_parameter("subscribe_address").get_parameter_value().string_value)
        self.subscription=self.create_subscription(
            Joy, subscribe_address, self.listener_callback, 10
        )

        self.declare_parameter("delta_time",0.1)
        self.declare_parameter("publish_address", "/ship1/control_input")
        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(Control, publish_address, 10)
        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)

    def listener_callback(self,data):
        self.n_p += (data.axes[2]-data.axes[5])*0.5
        self.rudder_angle_degree += -data.axes[0]*0.3

        if self.n_p<0:
            self.n_p=0.0
        elif self.n_p>120:
            self.n_p=120.0
        if self.rudder_angle_degree<-45:
            self.rudder_angle_degree=-45.0
        elif self.rudder_angle_degree>45:
            self.rudder_angle_degree=45.0


    def sender_callback(self):
        msg=Control()
        msg.n_p=self.n_p
        msg.rudder_angle_degree=self.rudder_angle_degree
        self.publisher.publish(msg)
        self.get_logger().info('`Publish: n_p="%s", rudder_angle="%s"'%(self.msg.n_p,self.msg.rudder_angle_degree))


def main(args=None):
    """Run main."""
    rclpy.init(args=args)
    node=MmgControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()