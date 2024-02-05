import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray
from shipsim_msgs_module.msg import PPguid
import numpy as np
import cmath
from sympy import geometry as sg

class PurepursuitNode(Node):
    def __init__(self, ship_number):
        super().__init__('purepursuit')
        self.ship_number = ship_number
        self.n_p=0.0
        self.rudder_angle_degree=0.0
        self.ship_x=0.0
        self.ship_y=0.0
        self.ship_psi=0.0
        self.now_wp_id=0
        self.declare_parameter("delta_time",0.1)

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
        
        self.declare_parameter("publish_address","/ship"+str(self.ship_number)+"/guidance")
        publish_address=self.get_parameter("publish_address").get_parameter_value().string_value
        self.pub_guide_angle=self.create_publisher(PPguid,publish_address,10)
        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time,self.sender_callback)
    
    def listener_callback1(self,msg):
        N=len(msg.data)//2
        self.wp_info=[]
        for i in range(N):
            self.wp_info.append((msg.data[2*i],msg.data[2*i+1]))
        self.get_logger().info('ship_number[%s] Subscribe: WP_info=%s'%(self.ship_number,self.wp_info))

    def listener_callback2(self,data):
        self.ship_x=data.linear.x
        self.ship_y=data.linear.y
        self.ship_psi=data.angular.z
        #self.get_logger().info('ship_number[%s] Subscribe: x="%s", y="%s"'%(self.ship_number, self.ship_x, self.ship_y))

    def sender_callback(self):
        self.msg=PPguid()
        pure_pursuit=Purepursuit(ShipPosition=(self.ship_x,self.ship_y,self.ship_psi), WayPoints=self.wp_info, now_wp_index=self.now_wp_id)
        self.msg.alpha_t,self.msg.track_error,self.now_wp_id=pure_pursuit.main()
        self.pub_guide_angle.publish(self.msg)
        self.get_logger().info('ship_number[%s] Publish: PP_angle=%s, Track Error=%s, Now Target Point=%s'%(self.ship_number,self.msg.alpha_t, self.msg.track_error, self.wp_info[self.now_wp_id]))

def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()
    num_of_ships = 1
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = PurepursuitNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

class Purepursuit:
    def __init__(self, ShipPosition, WayPoints, now_wp_index):
        self.ship=ShipPosition
        self.WPs=WayPoints
        self.now_wp_id=now_wp_index
        self.L_pp=7.00
        self.R_wp=3*self.L_pp
        self.L_t=3*self.L_pp

    def set_WayPoints(self):
        #現在目標としているWP(WPs[now_index])と船の距離がL未満であれば次のWPを目標とする
        #WPsの一番最後のWPの一定範囲内であれば、最初のWPに戻る
        M=len(self.WPs)
        s_x,s_y=self.ship[0],self.ship[1]
        k=self.now_wp_id
        now_WPx,now_WPy=self.WPs[k]
        dist=np.sqrt((s_x-now_WPx)**2+(s_y-now_WPy)**2)
        if dist<self.R_wp:
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
        self.WP1,self.WP2=self.set_WayPoints()
        ship_x,ship_y=self.ship[0],self.ship[1]
        a,b,c=self.liner_eq()
        error=(a*ship_x+b*ship_y+c)/np.sqrt(a**2+b**2)
        return error
    
    def get_target_point(self):
        self.WP1,self.WP2=self.set_WayPoints()
        error=self.calc_TrackError()
        self.psi=self.ship[2]
        if abs(error)<self.L_t:
            R_pp=self.L_t
        else:
            R_pp=self.L_t+abs(error)
        circle=sg.Circle(sg.Point(self.ship[0],self.ship[1]),R_pp)
        line=sg.Line(sg.Point(self.WP1[0],self.WP1[1]),sg.Point(self.WP2[0],self.WP2[1]))
        results=sg.intersection(circle,line)
        if len(results)==1:
            target_point=(float(results[0].x),float(results[0].y))
        else:
            t1=np.array([float(results[0].x),float(results[0].y)])
            t2=np.array([float(results[1].x),float(results[1].y)])
            dist1=np.linalg.norm(t1-self.WP2)
            dist2=np.linalg.norm(t2-self.WP2)
            if dist1<dist2:
                target_point=t1
            else:
                target_point=t2
        #print(target_point)
        return target_point
    
    def calc_ang(self):
        north=np.array([1,0])
        psi=self.ship[2]
        target_point=self.get_target_point()
        ship=np.array([self.ship[0],self.ship[1]])
        P=target_point-ship
        P_dot=np.dot(P,north)
        P_norm=np.linalg.norm(P)
        alpha=np.rad2deg(np.arccos(P_dot/P_norm))*np.sign(P[1])
        #print(alpha)
        ang=(alpha-psi)*np.pi/180
        ang=cmath.phase(cmath.exp(1j*ang))*180/np.pi
        return ang
    
    def main(self):
        track_error=self.calc_TrackError()
        ang=self.calc_ang()
        return ang,track_error, self.now_wp_id