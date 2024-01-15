import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from shipsim_msgs_module.msg import PPguid
from rclpy.executors import SingleThreadedExecutor
import numpy as np
from sympy import geometry as sg
from numpy import linalg as LA


class PPguidanceNode(Node):
    def __init__(self, ship_number):
        super().__init__('guidance')
        self.ship_number=ship_number
        self.now_wp_id=0
        self.ship_x=0.0
        self.ship_y=0.0

        self.declare_parameter("subscribe_address1","/ship"+str(self.ship_number)+"/wp_info")
        subscribe_address1=self.get_parameter("subscribe_address1").get_parameter_value().string_value
        self.subscription1=self.create_subscription(
            Float64MultiArray, subscribe_address1, self.listener_callback1, 10
            )

        self.declare_parameter("subscribe_address2","/ship"+str(self.ship_number)+"/obs_pose")
        subscribe_address2=self.get_parameter("subscribe_address2").get_parameter_value().string_value
        self.subscription2=self.create_subscription(
            Twist, subscribe_address2, self.listener_callback2, 10
            )

        self.declare_parameter("delta_time",1.0)
        self.declare_parameter("publish_address", "/ship"+str(self.ship_number)+"/guidance")
        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.pub_guide_angle = self.create_publisher(PPguid, publish_address, 10)

        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)

    def listener_callback1(self,msg):
        N=len(msg.data)//2
        self.wp_info=[]
        for i in range(N):
            self.wp_info.append((msg.data[2*i],msg.data[2*i+1]))
        self.get_logger().info('ship_number[%s] Subscribe: WP_info=%s'%(self.ship_number,self.wp_info))

    def listener_callback2(self,data):
        self.ship_x=data.linear.x
        self.ship_y=data.linear.y
        self.get_logger().info('ship_number[%s] Subscribe: x="%s", y="%s"'%(self.ship_number, self.ship_x, self.ship_y))
    
    def sender_callback(self):
        self.msg=PPguid()
        pp_guid=0.0 #TODO: calculate PPangle
        self.msg.pp_angle, self.now_wp_id = pp_guid.desired_angle()
        self.pub_guide_angle.publish(self.msg)
        self.get_logger().info('ship_number[%s] Publish: PP_angle=%s'%(self.ship_number,self.msg.alpha_t))

def main(args=None):
    """Run main"""
    rclpy.init(args=args)

    exec = SingleThreadedExecutor()
    num_of_ships = 2
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]

    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = PPguidanceNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()


class Purepursuit:
    def __init__(self,ShipPosition, WayPoints, now_wp_index, L_pp):
        self.ShipPosition=ShipPosition
        self.WayPoints=WayPoints
        self.now_wp_index=now_wp_index
        self.L_pp=L_pp