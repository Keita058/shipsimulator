import sys
import rclpy
from rclpy.node import Node
from shipsim_msgs_module.msg import Control
from sensor_msgs.msg import Joy
from rclpy.executors import SingleThreadedExecutor

class MmgControllerNode(Node):
    """ControllerNode."""
    def __init__(self,ship_number):
        """init."""
        super().__init__("controller")
        self.ship_number=ship_number
        self.n_p=0.0
        self.rudder_angle_degree=0.0
        self.declare_parameter("subscribe_address","/joy"+str(ship_number))
        subscribe_address=(self.get_parameter("subscribe_address").get_parameter_value().string_value)
        self.subscription=self.create_subscription(
            Joy, subscribe_address, self.listener_callback, 10
        )

        self.declare_parameter("delta_time",0.1)
        self.declare_parameter("publish_address", "/ship"+str(ship_number)+"/control_input")
        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(Control, publish_address, 10)

        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)

    def listener_callback(self,data):
        if data.buttons[6] or data.buttons[7]:
            self.n_p += (data.axes[2]-data.axes[5])*0.01
        self.n_p +- (data.buttons[2]*5-data.buttons[0]*5)

        self.rudder_angle_degree += -data.axes[0]*0.01
        self.rudder_angle_degree += (data.buttons[1]*5-data.buttons[3]*5)

        if self.n_p<0:
            self.n_p=0.0
        elif self.n_p>30:
            self.n_p=30.0
        if self.rudder_angle_degree<-45:
            self.rudder_angle_degree=-45.0
        elif self.rudder_angle_degree>45:
            self.rudder_angle_degree=45.0


    def sender_callback(self):
        self.msg=Control()
        self.msg.n_p=self.n_p
        self.msg.rudder_angle_degree=self.rudder_angle_degree
        self.publisher.publish(self.msg)
        self.get_logger().info('`ship_number[%s]Publish: n_p=%s, rudder_angle=%s'%(self.ship_number,self.msg.n_p,self.msg.rudder_angle_degree))


def main(args=None):
    """Run main."""
    rclpy.init(args=args)

    exec = SingleThreadedExecutor()
    num_of_ships = 1
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = MmgControllerNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()