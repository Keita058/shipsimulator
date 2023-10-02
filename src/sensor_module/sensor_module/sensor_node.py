import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from shipsim_msgs_module.msg import Control
from rclpy.executors import SingleThreadedExecutor

class SensorNode(Node):
    def __init__(self, ship_number):
        super().__init__("sensor")
        self.ship_number=ship_number

        self.declare_parameter("subscribe_address1", "/ship"+str(ship_number)+"/obs_pose")
        subscribe_address1=(self.get_parameter("subscribe_address1").get_parameter_value().string_value)
        self.subscription = self.create_subscription(
            Twist, subscribe_address1, self.listener_callback1, 10
        )

        self.declare_parameter("subscribe_address2", "/ship"+str(ship_number)+"/obs_vel")
        subscribe_address2=(self.get_parameter("subscribe_address2").get_parameter_value().string_value)
        self.subscription = self.create_subscription(
            Twist, subscribe_address2, self.listener_callback2, 10
        )

    def listener_callback1(self,msg):
        self.get_logger().info('ship_number[%s] Subscribe: x="%ss", y="%s", psi="%s"'%(self.ship_number, msg.linear.x, msg.linear.y, msg.angular.z))

    def listener_callback2(self,msg):
        self.get_logger().info('ship_number[%s] Subscribe: u="%ss", v="%s", r="%s"'%(self.ship_number, msg.linear.x, msg.linear.y, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)

    exec = SingleThreadedExecutor()
    num_of_ships = 2
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = SensorNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()

    rclpy.shutdow()

if __name__=="__main__":
    main()