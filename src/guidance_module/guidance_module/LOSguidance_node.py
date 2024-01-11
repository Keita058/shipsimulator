import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rclpy.executors import SingleThreadedExecutor
from shipsim_msgs_module.msg import LOSangle
import numpy as np
from sympy import geometry as sg
from numpy import linalg as LA

class LosguidanceNode(Node):
    """LOSGuidanceNode"""
    def __init__(self, ship_number):
        super().__init__("guidance")
        self.ship_number=ship_number
        self.now_wp_id=0
        self.ship_x=0.0
        self.ship_y=0.0

        self.declare_parameter("subscribe_address1","/ship"+str(self.ship_number)+"/wp_info")
        subscribe_address1=self.get_parameter("subscribe_address1").get_parameter_value().string_value
        self.subscription1=self.create_subscription(
            Float64MultiArray, subscribe_address1, self.listener_callback1, 10
            )

        self.declare_parameter("subscribe_address2","/ship"+str(self.ship_number)+"/obs_pose")
        subscribe_address2=self.get_parameter("subscribe_address2").get_parameter_value().string_value
        self.subscription2=self.create_subscription(
            Twist, subscribe_address2, self.listener_callback2, 10
            )

        self.declare_parameter("delta_time",1.0)
        self.declare_parameter("publish_address", "/ship"+str(self.ship_number)+"/guidance")
        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.pub_guide_angle = self.create_publisher(LOSangle, publish_address, 10)

        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)

    def listener_callback1(self,msg):
        N=len(msg.data)//2
        self.wp_info=[]
        for i in range(N):
            self.wp_info.append((msg.data[2*i],msg.data[2*i+1]))
        self.get_logger().info('ship_number[%s] Subscribe: WP_info=%s'%(self.ship_number,self.wp_info))

    def listener_callback2(self,data):
        self.ship_x=data.linear.x
        self.ship_y=data.linear.y
        self.get_logger().info('ship_number[%s] Subscribe: x="%s", y="%s"'%(self.ship_number, self.ship_x, self.ship_y))


    def sender_callback(self):
        self.msg=LOSangle()
        los_guid=LOSGuidance(ShipPosition=(self.ship_x,self.ship_y), WayPoints=self.wp_info, now_wp_index=self.now_wp_id, L_pp=7.0)
        self.msg.los_angle, self.now_wp_id = los_guid.desired_angle() #TODO: calculate LOSangle
        self.msg.track_error = los_guid.calc_TrackError()
        self.pub_guide_angle.publish(self.msg)
        self.get_logger().info('`ship_number[%s]Publish: LOSangle=%s, TrackError=%s, Now Target Point=%s'%(self.ship_number,self.msg.los_angle*180/np.pi, self.msg.track_error, self.wp_info[self.now_wp_id]))

def main(args=None):
    """Run main"""
    rclpy.init(args=args)

    exec = SingleThreadedExecutor()
    num_of_ships = 2
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]

    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = LosguidanceNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

class LOSGuidance:
    def __init__(self, ShipPosition, WayPoints, now_wp_index,L_pp):
        self.ship=ShipPosition
        self.WPs=WayPoints
        self.now_wp_id=now_wp_index
        self.L_pp=L_pp
        self.LOS_radius=3*self.L_pp

    def set_WayPoints(self):
        #現在目標としているWP(WPs[now_index])と船の距離がL未満であれば次のWPを目標とする
        #WPsの一番最後のWPの一定範囲内であれば、最初のWPに戻る
        M=len(self.WPs)
        s_x,s_y=self.ship[0],self.ship[1]
        k=self.now_wp_id
        now_WPx,now_WPy=self.WPs[k]
        dist=np.sqrt((s_x-now_WPx)**2+(s_y-now_WPy)**2)
        if dist<3*self.L_pp:
            k+=1
        k=k%M
        k_prev=(k-1)%M
        res_WP1,res_WP2=self.WPs[k_prev],self.WPs[k]
        self.now_wp_id=k
        return res_WP1,res_WP2

    def liner_eq(self):
        #P1,P2はtuple型
        #P1=(x1,y1),P2=(x2,y2)
        #ax+by+c=0型
        x1,y1=self.WP1
        x2,y2=self.WP2
        a=y2-y1
        b=x1-x2
        c=x2*y1-x1*y2
        return (a,b,c)

    def calc_TrackError(self):
        ship_x,ship_y=self.ship
        a,b,c=self.liner_eq()
        error=abs(a*ship_x+b*ship_y+c)/np.sqrt(a**2+b**2)
        return error

    def calc_LOSRadius(self):
        if self.track_error<self.LOS_radius:
            R=self.LOS_radius
        else:
            R=self.track_error+self.LOS_radius
        return R

    def calc_HeadingAngle(self):
        #船の現在地とP1,P2,WP2の位置関係からLOS Pointを選択
        #LOS Pointと船の現在地からHeadingAngleを計算
        north=np.array([1,0])
        P1=np.array([float(self.intersection1.x),float(self.intersection1.y)])
        P2=np.array([float(self.intersection2.x),float(self.intersection2.y)])
        WP=self.WP2
        WP=np.array([WP[0],WP[1]])
        Ship=np.array([self.ship[0],self.ship[1]])
        A=P1-Ship
        B=P2-Ship
        dist1=LA.norm(P1-WP)
        dist2=LA.norm(P2-WP)
        #print('vector A,B,C=',A,B,C)
        #正であるほうが次のWPに近い点
        if dist1<dist2:
            #P1と北がなす角を返す
            res_ang=np.arctan2(A[1],A[0])
        else:
            #P2と北がなす角を返す
            res_ang=np.arctan2(B[1],B[0])
        if res_ang>np.pi:
            res_ang=res_ang-2*np.pi
        return res_ang

    def desired_angle(self):
        self.WP1,self.WP2=self.set_WayPoints()
        self.track_error=self.calc_TrackError()
        R_LOS=self.calc_LOSRadius()
        circle=sg.Circle(sg.Point(self.ship[0],self.ship[1]),R_LOS)
        line=sg.Line(sg.Point(self.WP1),sg.Point(self.WP2))
        results=sg.intersection(circle,line)
        if len(results)==1:
            self.intersection1=results[0]
            self.intersection2=self.intersection1
        else:
            self.intersection1,self.intersection2=results[0],results[1]
        d_ang=self.calc_HeadingAngle()
        return d_ang, self.now_wp_id