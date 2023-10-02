import rclpy
from rclpy.node import Node
from shipsim_msgs_module.msg import Control
from rclpy.executors import SingleThreadedExecutor

class ActuatorNode(Node):

    def __init__(self, ship_number):
        super().__init__("actuator")
        self.ship_number=ship_number
        self.n_p=0.0
        self.current_n_p=0.0
        self.rudder_angle_degree=0.0
        self.current_angle=0.0
        self.actuated_list={'n_p':[0.0],'rudder_angle':[0.0]}

        self.declare_parameter("subscribe_address","/ship"+str(ship_number)+"/control_input")
        subscribe_address=(self.get_parameter("subscribe_address").get_parameter_value().string_value)
        self.subscription=self.create_subscription(
            Control, subscribe_address, self.listener_callback, 10
        )
        self.declare_parameter("sub_delta_time", 0.1)

        self.declare_parameter("publish_address","/ship"+str(ship_number)+"/cmd_input")
        self.declare_parameter("delta_time",0.1)
        publish_address=(self.get_parameter("publish_address").get_parameter_value().string_value)
        self.pub_actuator=self.create_publisher(Control, publish_address, 10)
        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)

    def listener_callback(self,msg):
        self.get_logger().info('ship_nunmber[%s] Subscribe: n_p="%s", rudder_angle="%s"'%(self.ship_number,msg.n_p,msg.rudder_angle_degree))
        self.n_p=msg.n_p
        self.rudder_angle_degree=msg.rudder_angle_degree

    def sender_callback(self):
        self.pub_actuator_msg=Control()

        sub_delta_time=self.get_parameter("sub_delta_time").value
        self.pub_actuator_msg=self.actuated(self.n_p, self.rudder_angle_degree,sub_delta_time)

        self.pub_actuator.publish(self.pub_actuator_msg)
        self.get_logger().info('ship_number[%s] Publish: n_p="%s", rudder_angle="%s"'%(self.ship_number, self.pub_actuator_msg.n_p, self.pub_actuator_msg.rudder_angle_degree))

    def actuated(self,n_p,rudder_angle,delta_time):
        actuated_msg=Control()
        res_n_p=res_angle=0.0
        rudder_rate=5.0
        n_p_rate=5.0

        current_n_p=self.actuated_list['n_p'][-1]
        d_n_p=n_p-current_n_p
        if abs(d_n_p)<n_p_rate*delta_time:
            res_n_p=n_p
        else:
            if d_n_p>0:
                res_n_p=current_n_p+n_p_rate*delta_time
            else:
                res_n_p=current_n_p-n_p_rate*delta_time

        current_angle=self.actuated_list['rudder_angle'][-1]
        d_angle=rudder_angle-current_angle
        if abs(d_angle)<rudder_rate*delta_time:
            res_angle=rudder_angle
        else:
            if d_angle>0:
                res_angle=current_angle+rudder_rate*delta_time
            else:
                res_angle=current_angle-rudder_rate*delta_time

        self.actuated_list['n_p'].append(res_n_p)
        self.actuated_list['rudder_angle'].append(res_angle)
        actuated_msg.n_p=res_n_p
        actuated_msg.rudder_angle_degree=res_angle
        return actuated_msg


def main(args=None):
    """Run main."""
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()
    num_of_ships = 2
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = ActuatorNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()

    rclpy.shutdown()

if __name__=="__main__":
    main()