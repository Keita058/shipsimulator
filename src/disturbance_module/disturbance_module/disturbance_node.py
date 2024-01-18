import rclpy
from rclpy.node import Node
from shipsim_msgs_module.msg import Disturbance
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.executors import SingleThreadedExecutor

class DisturabnceNode(Node):
    def __init__(self,ship_number):
        super().__init__("disturbance")
        self.ship_number=ship_number

        self.declare_parameter("delta_time",1.0)
        self.declare_parameter("X_d_0",10.0)
        self.declare_parameter("Y_d_0",10.0)
        self.declare_parameter("sigma_X_d",0.0)
        self.declare_parameter("sigma_Y_d",0.0)

        self.declare_parameter("subscribe_address", "/ship"+str(ship_number)+"/obs_pose")
        subscribe_address=(self.get_parameter("subscribe_address").get_parameter_value().string_value)
        self.create_subscription(Twist,subscribe_address,self.listener_callback,10)

        self.declare_parameter("publish_address", "/ship"+str(ship_number)+"/disturbance")
        publish_address=(self.get_parameter("publish_address").get_parameter_value().string_value)
        self.pub_disturbance=self.create_publisher(Disturbance,publish_address,10)
        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time,self.sender_callback)

    def listener_callback(self,msg):
        self.get_logger().info('ship_number[%s] Subscribe: x="%s", y="%s", psi="%s"'%(self.ship_number, msg.linear.x, msg.linear.y, msg.angular.z))
        self.x=msg.linear.x
        self.y=msg.linear.y
        self.psi=msg.angular.z
    
    def sender_callback(self):
        msg=Disturbance()
        msg.x_d,msg.y_d = self.get_disturbance()
        self.pub_disturbance.publish(msg)
        self.get_logger().info('ship_number[%s] Publish: X_d=%s, Y_d=%s'%(self.ship_number, msg.X_d, msg.Y_d))

    def get_disturbance(self):
        X_d_0=self.get_parameter("X_d_0").value
        Y_d_0=self.get_parameter("Y_d_0").value
        sigma_X_d=self.get_parameter("sigma_X_d").value
        sigma_Y_d=self.get_parameter("sigma_Y_d").value
        X_d=np.random.normal(X_d_0,sigma_X_d)
        Y_d=np.random.normal(Y_d_0,sigma_Y_d)
        #船体座標系への変換
        X_d_ship=X_d*np.cos(self.psi)-Y_d*np.sin(self.psi)
        Y_d_ship=X_d*np.sin(self.psi)+Y_d*np.cos(self.psi)

        return X_d_ship,Y_d_ship

def main(args=None):
    rclpy.init(args=args)

    exec = SingleThreadedExecutor()
    num_of_ships = 1
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = DisturabnceNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()

    rclpy.shutdown()

if __name__=="__main__":
    main()