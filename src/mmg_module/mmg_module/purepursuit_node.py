import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from shipsim_msgs_module.msg import Control
import numpy as np
import cmath

class PurepursuitNode(Node):
    def __init__(self, ship_number):
        super().__init__('purepursuit')
        self.ship_number = ship_number
        self.n_p=0.0
        self.rudder_angle_degree=0.0
        self.declare_parameter("delta_time",1.0)

        self.declare_parameter("subscribe_address1","/ship"+str(self.ship_number)+"/obs_pose")
        subscribe_address1=self.get_parameter("subscribe_address1").get_parameter_value().string_value
        self.subscription1=self.create_subscription(
            Twist, subscribe_address1, self.listener_callback1, 10
            )
        self.declare_parameter("subscribe_address2","/ship"+str(self.ship_number)+"/obs_vel")
        subscribe_address2=self.get_parameter("subscribe_address2").get_parameter_value().string_value
        self.subscription2=self.create_subscription(
            Twist, subscribe_address2, self.listener_callback2, 10
            )
        
        self.declare_parameter("publish_address","/ship"+str(self.ship_number)+"/control_input")
        publish_address=self.get_parameter("publish_address").get_parameter_value().string_value
        self.pub_cmd_vel=self.create_publisher(Twist, publish_address, 10)
        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)
    
    def listener_callback1(self, data):
        self.ship_x=data.linear.x
        self.ship_y=data.linear.y
        if data.angular.z>180:
            self.ship_yaw=(data.angular.z-360)*np.pi/180
        else:
            self.ship_yaw=data.angular.z*np.pi/180
        self.get_logger().info('ship_number[%s] Subscribe: x="%s", y="%s", yaw="%s"'%(self.ship_number, self.ship_x, self.ship_y, self.ship_yaw))

    def listener_callback2(self, data):
        self.ship_u=data.linear.x
        self.ship_v=data.linear.y
        self.ship_r=data.angular.z
        self.get_logger().info('ship_number[%s] Subscribe: u="%s", v="%s", r="%s"'%(self.ship_number, self.ship_u, self.ship_v, self.ship_r))

    def sender_callback(self):
        self.msg=Control()
        delta_time=self.get_parameter("delta_time").value
        self.msg.n_p, self.msg.rudder_angle_degree = self.pure_pursuit()

    def pure_pursuit(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()
    num_of_ships = 2
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = PurepursuitNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
