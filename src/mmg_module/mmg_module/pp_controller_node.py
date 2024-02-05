import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from shipsim_msgs_module.msg import PPguid
from shipsim_msgs_module.msg import Control
import numpy as np
import cmath

class PPControllerNode(Node):
    def __init__(self, ship_number):
        super().__init__('pp_controller')
        self.ship_number = ship_number
        self.e_int=0.0
        self.e_prev=0.0
        #Publish Parameter
        self.n_p=10.0
        self.rudder_angle_degree=0.0
        self.ship_u=0.0
        self.ship_yaw=0.0
        self.alpha_t=0.0

        #Subscriber
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
        self.declare_parameter("subscribe_address3","/ship"+str(self.ship_number)+"/guidance")
        subscribe_address3=self.get_parameter("subscribe_address3").get_parameter_value().string_value
        self.subscription3=self.create_subscription(
            PPguid, subscribe_address3, self.listener_callback3, 10
            )
        
        self.declare_parameter("subscribe_address4","/ship"+str(self.ship_number)+"/cmd_input")
        subscribe_address4=self.get_parameter("subscribe_address4").get_parameter_value().string_value
        self.subscription4=self.create_subscription(
            Control, subscribe_address4, self.listener_callback4, 10
            )
        
        #Publisher
        self.declare_parameter("delta_time",0.1)
        self.declare_parameter("publish_address","/ship"+str(self.ship_number)+"/control_input")
        publish_address=self.get_parameter("publish_address").get_parameter_value().string_value
        self.pub_control=self.create_publisher(Control, publish_address, 10)
        self.delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(self.delta_time, self.sender_callback)
    
    def listener_callback1(self,data):
        self.ship_x=data.linear.x
        self.ship_y=data.linear.y
        if data.angular.z>180:
            self.ship_yaw=(data.angular.z-360)
        else:
            self.ship_yaw=data.angular.z
        self.ship_yaw=self.ship_yaw*np.pi/180
        #self.get_logger().info('ship_number[%s] Subscribe: x="%s", y="%s", psi="%s"'%(self.ship_number, self.ship_x, self.ship_y, self.ship_yaw))
    
    def listener_callback2(self,data):
        self.ship_u=data.linear.x
        self.ship_v=data.linear.y
        self.ship_r=data.angular.z
        #self.get_logger().info('ship_number[%s] Subscribe: u="%s", r="%s"'%(self.ship_number, self.ship_u, self.ship_r))
    
    def listener_callback3(self,data):
        self.alpha_t=data.alpha_t*np.pi/180
        self.track_error=data.track_error
        #self.get_logger().info('ship_number[%s] Subscribe: alpha_t="%s", track_error="%s"'%(self.ship_number, self.alpha_t, self.track_error))
    
    def listener_callback4(self,data):
        self.n_p=data.n_p
        self.delta=data.rudder_angle_degree
        #self.get_logger().info('ship_number[%s] Subscribe: delta="%s"'%(self.ship_number, self.delta))

    def sender_callback(self):
        self.msg=Control()
        self.msg.n_p,self.msg.rudder_angle_degree=self.pp_controller()
        self.pub_control.publish(self.msg)
        self.get_logger().info('ship_number[%s] Publish: n_p=%s, rudder_angle=%s'%(self.ship_number, self.msg.n_p, self.msg.rudder_angle_degree))
    
    def pp_controller(self):
        KP_ang=100.0
        KD_ang=0.0
        next_rudder=KP_ang*self.alpha_t-KD_ang*self.ship_yaw
        if next_rudder<-45:
            next_rudder=-45.0
        elif next_rudder>45:
            next_rudder=45.0
        u_ref=1.5
        e_u=u_ref-self.ship_u
        e_dot=(e_u-self.e_prev)/self.delta_time
        self.e_prev=e_u
        self.e_int=self.e_int+e_u*self.delta_time
        KP_u=1.0
        KI_u=0.1
        KD_u=10
        next_np=KP_u*e_u+KI_u*self.e_int+KD_u*e_dot+self.n_p
        if next_np<10:
            next_np=10.0
        elif next_np>50:
            next_np=50.0
        return next_np, next_rudder

def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()
    num_of_ships = 1
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = PPControllerNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()