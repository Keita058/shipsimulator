import rclpy
from rclpy.node import Node
from shipsim_msgs_module.msg import Control

class ActuatorNode(Node):

    n_p=0.0
    current_n_p=0.0
    rudder_angle_degree=0.0
    current_angle=0.0

    actuated_list={'n_p':[current_n_p],'rudder_angle':[current_angle]}

    def __init__(self):
        super().__init__("actuator",namespace="ship1")

        self.declare_parameter("subscribe_address","/ship1/control_input")
        subscribe_address=(self.get_parameter("subscribe_address").get_parameter_value().string_value)
        self.subscription=self.create_subscription(
            Control, subscribe_address, self.listener_callback, 10
        )
        self.declare_parameter("sub_delta_time", 0.1)

        self.declare_parameter("publish_address","/ship1/cmd_input")
        self.declare_parameter("delta_time",0.01)
        publish_address=(self.get_parameter("publish_address").get_parameter_value().string_value)
        self.pub_actuator=self.create_publisher(Control, publish_address, 10)
        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)

    def listener_callback(self,msg):
        self.get_logger().info('Subscribe: n_p="%s", rudder_angle="%s"'%(msg.n_p,msg.rudder_angle_degree))
        self.n_p=msg.n_p
        self.rudder_angle_degree=msg.rudder_angle_degree

    def sender_callback(self):
        self.pub_actuator_msg=Control()

        sub_delta_time=self.get_parameter("sub_delta_time").value
        self.pub_actuator_msg=self.actuated(self.n_p, self.rudder_angle_degree,sub_delta_time)

        self.pub_actuator.publish(self.pub_actuator_msg)
        self.get_logger().info('Publish: n_p="%s", rudder_angle="%s"'%(self.pub_actuator_msg.n_p, self.pub_actuator_msg.rudder_angle_degree))

    def actuated(self,n_p,rudder_angle,delta_time):
        actuated_msg=Control()
        res_n_p=res_angle=0.0

        current_n_p=self.actuated_list['n_p'][-1]
        d_n_p=n_p-current_n_p
        if abs(d_n_p)<0.5*delta_time:
            res_n_p=n_p
        else:
            if d_n_p>0:
                res_n_p=current_n_p+0.5*delta_time
            else:
                res_n_p=current_n_p-0.5*delta_time

        current_angle=self.actuated_list['rudder_angle'][-1]
        d_angle=rudder_angle-current_angle
        if abs(d_angle)<0.5*delta_time:
            res_angle=rudder_angle
        else:
            if d_angle>0:
                res_angle=current_angle+0.5*delta_time
            else:
                res_angle=current_angle-0.5*delta_time

        self.actuated_list['n_p'].append(res_n_p)
        self.actuated_list['rudder_angle'].append(res_angle)
        actuated_msg.n_p=res_n_p
        actuated_msg.rudder_angle_degree=res_angle
        return actuated_msg


def main(args=None):
    rclpy.init(args=args)
    node=ActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()