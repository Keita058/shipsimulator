import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from shipsim_msgs_module.msg import LOSangle
import numpy as np
from sympy import geometry as sg

class LOSGuidance():
    def __init__(self, ship_position, WayPoint1, WayPoint2):
        self.ship=ship_position
        self.WP1=WayPoint1
        self.WP2=WayPoint2

    def linear_eq(self, WP1, WP2):
        WP1_x,WP1_y=WP1
        WP2_x,WP2_y=WP2
        #calc ax+by+c=0
        a=WP2_y-WP1_y
        b=-(WP2_x-WP1_x)
        c=WP2_x*WP1_y-WP1_x*WP2_y
        return (a,b,c)

    def track_error(self, ship_position, WP1, WP2):
        ship_x,ship_y=ship_position
        a,b,c=self.linear_eq(WP1, WP2)
        error=abs(a*ship_x+b*ship_y+c)/np.sqrt(a**2+b**2)
        return error

    def calc_intersection(self, center, radius, WP1, WP2):
        circle=sg.Circle(sg.Point(center), radius)
        line=sg.Line(sg.Point(WP1), sg.Point(WP2))
        result=sg.intersection(circle, line)
        return result

    def desired_angle(self):
        L_pp=7.00 #Ship Length
        N=3 #N:= R_LOS=N*L_pp
        line_coef=self.linear_eq(self.WP1, self.WP2)
        e=self.track_error(self.ship, line_coef)
        if e<N*L_pp:
            R_LOS=N*L_pp
        else:
            R_LOS=e+L_pp
        

class GuidanceNode(Node):
    """GuidanceNode"""
    def __init__(self, ship_number):
        super().__init__("guidance")
        self.ship_x=0.0
        self.ship_y=0.0

        self.declare_parameter("subscribe_address","/ship"+str(ship_number)+"/obs_pose")
        subscribe_address=(self.get_parameter("subscribe_address").get_parameter_value().string_value)
        self.subscription=self.create_subscription(
            Twist, subscribe_address, self.listener_callback, 10
        )

        self.declare_parameter("delta_time",0.1)
        self.declare_parameter("publish_address", "/ship"+str(ship_number)+"/guidance")
        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(Twist, publish_address, 10)

        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)

    def listener_callback(self,data):
        self.ship_x=data.linear.x
        self.ship_y=data.linear.y

    def sender_callback(self):
        self.msg=LOSangle()
        self.msg.los_angle=0.0 #TODO: calculate LOSangle
        self.publisher.publish(self.msg)
        self.get_logger().info('`ship_number[%s]Publish: LOSangle=%s'%(self.ship_number,self.msg.los_angle))

def main(args=None):
    """Run main"""
    rclpy.init(args=args)

    exec = SingleThreadedExecutor()
    num_of_ships = 2
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]

    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = GuidanceNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()