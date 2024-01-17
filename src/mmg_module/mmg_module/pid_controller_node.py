import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from shipsim_msgs_module.msg import LOSangle
from shipsim_msgs_module.msg import Control
import numpy as np
import cmath

class PIDControllerNode(Node):
    def __init__(self, ship_number):
        super().__init__('pid_controller')
        self.ship_number = ship_number
        self.e_int=np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.e_prev=np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        #Publish Parameter
        self.n_p=0.0
        self.rudder_angle_degree=0.0
        self.LOS_angle=0.0
        self.ship_u=0.0
        self.ship_yaw=0.0
        self.ship_r=0.0
        self.ship_r_prev=0.0

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
            LOSangle, subscribe_address3, self.listener_callback3, 10
            )
        
        self.declare_parameter("subscribe_address4","/ship"+str(self.ship_number)+"/cmd_input")
        subscribe_address4=self.get_parameter("subscribe_address4").get_parameter_value().string_value
        self.subscription4=self.create_subscription(
            Control, subscribe_address4, self.listener_callback4, 10
            )
        
        #Publisher
        self.declare_parameter("delta_time",1.0)
        self.declare_parameter("publish_address","/ship"+str(self.ship_number)+"/control_input")
        publish_address=self.get_parameter("publish_address").get_parameter_value().string_value
        self.pub_control=self.create_publisher(Control, publish_address, 10)
        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)

    def listener_callback1(self,data):
        self.ship_x=data.linear.x
        self.ship_y=data.linear.y
        if data.angular.z>180:
            self.ship_yaw=(data.angular.z-360)*np.pi/180
        else:
            self.ship_yaw=data.angular.z*np.pi/180
        #self.get_logger().info('ship_number[%s] Subscribe: x="%s", y="%s", yaw="%s"'%(self.ship_number, self.ship_x, self.ship_y, self.ship_yaw))
    
    def listener_callback2(self,data):
        self.ship_u=data.linear.x
        self.ship_v=data.linear.y
        self.ship_r=data.angular.z
        #self.get_logger().info('ship_number[%s] Subscribe: u="%s", v="%s", r="%s"'%(self.ship_number, self.ship_u, self.ship_v, self.ship_r))
    
    def listener_callback3(self,data):
        self.LOS_angle=data.los_angle
        self.track_error=data.track_error
        self.get_logger().info('ship_number[%s] Subscribe: LOS_angle="%s", TrackError=%s'%(self.ship_number, self.LOS_angle, self.track_error))

    def listener_callback4(self,data):
        self.n_p=data.n_p
        self.rudder_angle_degree=data.rudder_angle_degree
        #self.get_logger().info('ship_number[%s] Subscribe: n_p="%s", rudder_angle="%s"'%(self.ship_number, self.n_p, self.rudder_angle_degree))

    def sender_callback(self):
        self.msg=Control()
        delta_time=self.get_parameter("delta_time").value
        self.msg.n_p, self.msg.rudder_angle_degree = self.PIDController(self.LOS_angle, self.ship_yaw, self.ship_u, self.n_p, self.rudder_angle_degree, delta_time)
        self.pub_control.publish(self.msg)
        self.get_logger().info('`ship_number[%s]Publish: n_p=%s, rudder_angle_degree=%s'%(self.ship_number,self.msg.n_p,self.msg.rudder_angle_degree))
    
    def PIDController(self, LOS_angle, ship_yaw, ship_u, n_p, rudder_angle_degree, delta_time):
        u_ref=1.5
        e_u=u_ref-ship_u
        temp_z=cmath.exp((LOS_angle-ship_yaw)*1j)
        e_ang=cmath.phase(temp_z)*180/np.pi
        e_arr=np.array([e_u, e_ang, -self.track_error, -self.ship_r, -(self.ship_r-self.ship_r_prev)])
        self.ship_r_prev=self.ship_r
        K_P=np.array([[5, 0, 0, 0, 0],[0, 1, 1, 0, 0]])
        K_I=np.array([[10, 0, 0, 0, 0],[0, 0, 0, 0, 0]])
        K_D=np.array([[50, 0, 0, 0, 0],[0, 500, 0, 0, 0]])
        e_dot=(e_arr-self.e_prev)/delta_time
        e_int=self.e_int+e_arr*delta_time
        m=np.dot(K_P,e_arr)+np.dot(K_I,e_int)+np.dot(K_D,e_dot)+np.array([self.n_p, self.rudder_angle_degree])
        self.e_prev=e_arr
        if m[0]<0:
            n_p=0.0
        elif m[0]>30.0:
            n_p=30.0
        else:
            n_p=m[0]
        if m[1]<-45:
            rudder_angle_degree=-45.0
        elif m[1]>45:
            rudder_angle_degree=45.0
        else:
            rudder_angle_degree=m[1]
        return n_p, rudder_angle_degree


def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()
    num_of_ships = 1
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = PIDControllerNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

