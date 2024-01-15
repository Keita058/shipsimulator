import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from rclpy.executors import SingleThreadedExecutor

class WaypointNode(Node):
    def __init__(self,ship_number):
        super().__init__('waypoint')
        self.ship_number=ship_number

        self.declare_parameter("delta_time",1.0)
        self.declare_parameter("publish_address","/ship"+str(self.ship_number)+"/wp_info")
        publish_address=self.get_parameter("publish_address").get_parameter_value().string_value
        self.pub_wp_info=self.create_publisher(Float64MultiArray, publish_address, 10)
        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)
    
    def sender_callback(self):
        WP1=[0.0,0.0]
        WP2=[90.0,0.0]
        WP3=[145.0,65.0]
        WP4=[190.0,65.0]
        WP5=[230.0,45.0]
        WP6=[320.0,0.0]
        WP_info=[WP1,WP2,WP3,WP4,WP5,WP6]
        msg=Float64MultiArray()
        for i in range(len(WP_info)):
            for j in range(len(WP_info[i])):
                msg.data.append(WP_info[i][j])
        self.pub_wp_info.publish(msg)
        self.get_logger().info('ship_number[%s]Publish: WP_info=%s'%(self.ship_number,WP_info))

def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()
    num_of_ships = 1
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = WaypointNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()