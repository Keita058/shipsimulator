import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from shipsim_msgs_module.msg import MPCguid
from shipsim_msgs_module.msg import Control
import numpy as np
import do_mpc
import dataclasses
class MPCControllerNode(Node):
    def __init__(self,ship_number):
        super().__init__("mpc_controller")
        self.ship_number=ship_number
        #Publish Parameter
        self.n_p=0.0
        self.rudder_angle_degree=0.0
        self.LOS_angle=0.0
        self.track_error=0.0

        #Subscriber
        self.declare_parameter("subscribe_address1", "/ship"+str(ship_number)+"/obs_pose")
        subscribe_address1=(self.get_parameter("subscribe_address1").get_parameter_value().string_value)
        self.subscription1 = self.create_subscription(
            Twist, subscribe_address1, self.listener_callback1, 10
        )
        self.declare_parameter("subscribe_address2", "/ship"+str(ship_number)+"/obs_vel")
        subscribe_address2=(self.get_parameter("subscribe_address2").get_parameter_value().string_value)
        self.subscription2 = self.create_subscription(
            Twist, subscribe_address2, self.listener_callback2, 10
        )
        self.declare_parameter("subscribe_address3", "/ship"+str(ship_number)+"/guidance")
        subscribe_address3=(self.get_parameter("subscribe_address3").get_parameter_value().string_value)
        self.subscription3 = self.create_subscription(
            MPCguid, subscribe_address3, self.listener_callback3, 10
        )

        #Publisher
        self.declare_parameter("delta_time",1.0)
        self.declare_parameter("publish_address", "/ship"+str(ship_number)+"/cmd_input")
        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.pub_cmd_input = self.create_publisher(Control, publish_address, 10)
        delta_time=self.get_parameter("delta_time").value
        self.timer=self.create_timer(delta_time, self.sender_callback)
    
    def listener_callback1(self,msg):
        #self.get_logger().info('ship_number[%s] Subscribe: x="%ss", y="%s", psi="%s"'%(self.ship_number, msg.linear.x, msg.linear.y, msg.angular.z))
        self.x=msg.linear.x
        self.y=msg.linear.y
        self.psi=msg.angular.z*np.pi/180
    
    def listener_callback2(self,msg):
        #self.get_logger().info('ship_number[%s] Subscribe: u="%ss", v="%s", r="%s"'%(self.ship_number, msg.linear.x, msg.linear.y, msg.angular.z))
        self.u=msg.linear.x
        self.v=msg.linear.y
        self.r=msg.angular.z

    def listener_callback3(self,msg):
        #self.get_logger().info('ship_number[%s] Subscribe: LOS_angle="%s"'%(self.ship_number, msg.LOS_angle))
        self.LOS_angle=msg.los_angle
        self.track_error=msg.track_error
    
    def sender_callback(self):
        self.msg=Control()
        basic_params = get_KVLCC2_L7model_basic_params()
        mpc_params = set_mpc_params(
            n_horizon=5,
            t_step=1,
            n_robust=1,
            store_full_solution=True,
            δ_set=1e-7,
            n_p_set=1e-7,
            δ_max=45.0*np.pi/180,
            δ_min=-45.0*np.pi/180,
            n_p_max=25.0,
            n_p_min=0.0,
            u_max=10.0,
            u_min=0.0,
            v_max=5.0,
            v_min=-5.0,
            control_duration=10,
            model_type='continuous'
        )
        Pose=(self.x, self.y, self.psi)
        Vel=(self.u, self.v, self.r)
        mpc=MPC_Control(Pose=Pose, Vel=Vel, track_error=self.track_error, n_p=self.n_p, rudder_angle_degree=self.rudder_angle_degree,basic_params=basic_params,mpc_params=mpc_params)
        self.msg.rudder_angle_degree,self.msg.n_p=mpc.get_mpc_input() #TODO: calculate n_p, rudder_angle_degree by MPC
        self.n_p=self.msg.n_p
        self.rudder_angle_degree=self.msg.rudder_angle_degree
        self.get_logger().info('ship_number[%s] Publish: n_p="%s", rudder_angle="%s"'%(self.ship_number, self.n_p, self.rudder_angle_degree*180/np.pi))
        self.pub_cmd_input.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()
    num_of_ships = 1
    nodes = ["node"+str(ship_number) for ship_number in range(1,num_of_ships+1)]
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]] = MPCControllerNode(ship_number+1)
    for ship_number in range(num_of_ships):
        exec.add_node(globals()[nodes[ship_number]])
    exec.spin()
    exec.shutdown()
    for ship_number in range(num_of_ships):
        globals()[nodes[ship_number]].destroy_node()

    rclpy.shutdown()

if __name__=="__main__":
    main()

class MPC_Control:
    def __init__(self, Pose, Vel, track_error, n_p, rudder_angle_degree,basic_params,mpc_params):
        self.x_now, self.y_now, self.psi_now = Pose
        self.u_now, self.v_now, self.r_now = Vel
        self.LOS_angle = track_error
        self.n_p_m = n_p
        self.rudder_angle_degree_m = rudder_angle_degree
        self.basic_params = basic_params
        self.mpc_params = mpc_params
        self.u_ref=1.025

    def set_variables(self, model):
        #Parameters
        self.x=model.set_variable(var_type='_x',var_name='x',shape=(1,1))
        self.y=model.set_variable(var_type='_x',var_name='y',shape=(1,1))
        self.ψ=model.set_variable(var_type='_x',var_name='ψ',shape=(1,1))
        self.u=model.set_variable(var_type='_x',var_name='u',shape=(1,1))
        self.v=model.set_variable(var_type='_x',var_name='v',shape=(1,1))
        self.r=model.set_variable(var_type='_x',var_name='r',shape=(1,1))
        self.n_p=model.set_variable(var_type='_x',var_name='n_p',shape=(1,1))
        self.δ=model.set_variable(var_type='_x',var_name='δ',shape=(1,1))

        self.δ_set = model.set_variable(var_type='_u', var_name='δ_set', shape=(1,1)) #舵角
        self.n_p_set = model.set_variable(var_type='_u', var_name='n_p_set', shape=(1,1)) #プロペラ回転数

    def set_model_for_MPC(self,model, MMG):
        #MPCに使う運動方程式の設定
        u=self.u
        v=self.v
        r=self.r
        ψ=self.ψ
        δ_set=self.δ_set
        n_p_set=self.n_p_set
        δ=self.δ
        n_p=self.n_p

        m=self.basic_params.m
        m_y=self.basic_params.m_y
        m_x=self.basic_params.m_x
        I_zG=self.basic_params.I_zG
        J_z=self.basic_params.J_z
        x_G=self.basic_params.x_G

        model.set_rhs('u', (MMG.X_H(u,v,r)+MMG.X_R(u,v,r,δ,n_p)+MMG.X_P(u,v,r,δ,n_p)+(m+m_y)*v*r+x_G*m*r**2)/(m+m_x))
        model.set_rhs('v', (x_G**2*m**2*u*r-(MMG.N_H(u,v,r)+MMG.N_R(u,v,r,δ,n_p))*x_G*m+(MMG.Y_H(u,v,r)+MMG.Y_R(u,v,r,δ,n_p)-(m+m_x)*u*r)*(I_zG+J_z+x_G**2*m))/((I_zG+J_z+x_G**2*m)*(m+m_y)-x_G**2*m**2))
        model.set_rhs('r', (MMG.N_H(u,v,r)+MMG.N_R(u,v,r,δ,n_p)-x_G*m*((x_G**2*m**2*u*r-(MMG.N_H(u,v,r)+MMG.N_R(u,v,r,δ,n_p))*x_G*m+(MMG.Y_H(u,v,r)+MMG.Y_R(u,v,r,δ,n_p)-(m+m_x)*u*r)*(I_zG+J_z+x_G**2*m))/((I_zG+J_z+x_G**2*m)*(m+m_y)-x_G**2*m**2)+u*r))/(I_zG+J_z+x_G**2*m))
        model.set_rhs('x', u * np.cos(ψ) - v * np.sin(ψ))
        model.set_rhs('y', u * np.sin(ψ) + v * np.cos(ψ))
        model.set_rhs('ψ', r)
        model.set_rhs('δ',(δ-δ_set))
        model.set_rhs('n_p',(n_p-n_p_set))
    
    def set_evaluation_func(self,model, mpc):
        c1=10.0
        c2=1.0
        lterm=c1*(self.track_error)**2+c2*(model.x['u']-self.u_ref)**2
        mterm=lterm
        mpc.set_objective(mterm=mterm,lterm=lterm)

        mpc.set_rterm(
            δ_set = self.mpc_params.δ_set,
            n_p_set = self.mpc_params.n_p_set
        )
    
    def set_mpc_bounds(self, mpc):
        mpc.bounds['lower','_u', 'δ_set'] = self.mpc_params.δ_min
        mpc.bounds['upper','_u', 'δ_set'] = self.mpc_params.δ_max
        mpc.bounds['lower','_x','u'] = self.mpc_params.u_min
        mpc.bounds['upper','_x','u'] = self.mpc_params.u_max
        mpc.bounds['lower','_x','v'] = self.mpc_params.v_min
        mpc.bounds['upper','_x','v'] = self.mpc_params.v_max
        mpc.bounds['lower','_u','n_p_set'] = self.mpc_params.n_p_min
        mpc.bounds['upper','_u','n_p_set'] = self.mpc_params.n_p_max

    def get_mpc_input(self):
        #ここがmain関数になる
        model_type=self.mpc_params.model_type
        model = do_mpc.model.Model(model_type)
        MMG=MMG_Model()

        self.set_variables(model)
        self.set_model_for_MPC(model, MMG)
        model.setup()

        mpc=do_mpc.controller.MPC(model)

        n_horizon=self.mpc_params.n_horizon
        t_step=self.mpc_params.t_step
        n_robust=self.mpc_params.n_robust
        store_full_solution=self.mpc_params.store_full_solution

        setup_mpc = {
            'n_horizon': n_horizon,
            't_step': t_step,
            'n_robust': n_robust,
            'store_full_solution': store_full_solution,
        }
        mpc.set_param(**setup_mpc)

        self.set_evaluation_func(model, mpc)
        self.set_mpc_bounds(mpc)
        mpc.setup()

        simulator = do_mpc.simulator.Simulator(model)
        simulator.set_param(t_step=t_step)
        simulator.setup()

        if self.u_now<0.01:
            self.u_now=0.01
        if self.n_p_m<0.01:
            self.n_p_m=0.01
        x0=np.array([self.x_now, self.y_now, self.psi_now, self.u_now, self.v_now, self.r_now, self.n_p_m, self.rudder_angle_degree_m]).reshape(-1,1)
        simulator.x0=x0
        mpc.x0=x0
        mpc.set_initial_guess()

        for i in range(n_horizon):
            u0=mpc.make_step(x0)
            x0=simulator.make_step(u0)
        u0=(u0[0][0],u0[1][0])
        return u0


class MMG_Model:
    def __init__(self):
        basic_params = get_KVLCC2_L7model_basic_params()
        mmg_params = get_KVLCC2_L7model_mmg_params()

        self.ρ = basic_params.ρ
        self.L_pp = basic_params.L_pp
        self.B = basic_params.B
        self.d = basic_params.d
        self.nabla = basic_params.nabla
        self.x_G_dash = basic_params.x_G_dash
        self.C_b = basic_params.C_b
        self.D_p = basic_params.D_p
        self.H_R = basic_params.H_R
        self.A_R = basic_params.A_R
        self.t_P = basic_params.t_P
        self.w_P0 = basic_params.w_P0
        self.m_x_dash = basic_params.m_x_dash
        self.m_y_dash = basic_params.m_y_dash
        self.J_z_dash = basic_params.J_z_dash
        self.t_R = basic_params.t_R
        self.x_R = basic_params.x_R
        self.a_H = basic_params.a_H
        self.x_H = basic_params.x_H
        self.γ_R_minus = basic_params.γ_R_minus
        self.γ_R_plus = basic_params.γ_R_plus
        self.l_r_dash = basic_params.l_r_dash
        self.x_P_dash = basic_params.x_P_dash
        self.ϵ = basic_params.ϵ
        self.κ = basic_params.κ
        self.f_α = basic_params.f_α
        self.m = basic_params.m
        self.I_zG = basic_params.I_zG
        self.η = basic_params.η
        self.m_x = basic_params.m_x
        self.m_y = basic_params.m_y
        self.J_z = basic_params.J_z
        self.x_H = basic_params.x_H
        self.x_R = basic_params.x_R
        self.x_G = basic_params.x_G

        self.k_0 = mmg_params.k_0
        self.k_1 = mmg_params.k_1
        self.k_2 = mmg_params.k_2
        self.R_0_dash = mmg_params.R_0_dash
        self.X_vv_dash = mmg_params.X_vv_dash
        self.X_vr_dash = mmg_params.X_vr_dash
        self.X_rr_dash = mmg_params.X_rr_dash
        self.X_vvvv_dash = mmg_params.X_vvvv_dash
        self.Y_v_dash = mmg_params.Y_v_dash
        self.Y_r_dash = mmg_params.Y_r_dash
        self.Y_vvv_dash = mmg_params.Y_vvv_dash
        self.Y_vvr_dash = mmg_params.Y_vvr_dash
        self.Y_vrr_dash = mmg_params.Y_vrr_dash
        self.Y_rrr_dash = mmg_params.Y_rrr_dash
        self.N_v_dash = mmg_params.N_v_dash
        self.N_r_dash = mmg_params.N_r_dash
        self.N_vvv_dash = mmg_params.N_vvv_dash
        self.N_vvr_dash = mmg_params.N_vvr_dash
        self.N_vrr_dash = mmg_params.N_vrr_dash
        self.N_rrr_dash = mmg_params.N_rrr_dash
        self.R_0 = mmg_params.R_0


    def X_H(self,u,v,r):
        vm=v-self.x_G*r
        U=np.sqrt(u**2+vm**2)
        r_dash=r*self.L_pp/U
        vm_dash=vm/U
        return 0.5*self.ρ*self.L_pp*self.d*U**2*self.X_H_dash(vm_dash,r_dash)

    def Y_H(self,u,v,r):
        vm=v-self.x_G*r
        U=np.sqrt(u**2+vm**2)
        r_dash=r*self.L_pp/U
        vm_dash=vm/U
        return 0.5*self.ρ*self.L_pp*self.d*U**2*self.Y_H_dash(vm_dash,r_dash)

    def N_H(self,u,v,r):
        vm=v-self.x_G*r
        U=np.sqrt(u**2+vm**2)
        r_dash=r*self.L_pp/U
        vm_dash=vm/U
        return 0.5*self.ρ*self.L_pp**2*self.d*U**2*self.N_H_dash(vm_dash,r_dash)

    #斜航角ではなく横方向の速度の無次元化成分を使う？
    def X_H_dash(self,β,r_dash):
        return -self.R_0_dash+self.X_vv_dash*β**2+self.X_vr_dash*β*r_dash+self.X_rr_dash*r_dash**2+self.X_vvvv_dash*β**4

    def Y_H_dash(self,β,r_dash):
        return self.Y_v_dash*β+self.Y_r_dash*r_dash+self.Y_vvr_dash*β**2*r_dash+self.Y_vrr_dash*β*r_dash**2+self.Y_vvv_dash*β**3+self.Y_rrr_dash*r_dash**3

    def N_H_dash(self,β,r_dash):
        return self.N_v_dash*β+self.N_r_dash*r_dash+self.N_vvr_dash*β**2*r_dash+self.N_vrr_dash*β*r_dash**2+self.N_vvv_dash*β**3+self.N_rrr_dash*r_dash**3
    
    def X_R(self,u,v,r,δ,n_p):
        return -(1-self.t_R)*self.F_N(u,v,r,δ,n_p)*np.sin(δ)

    def Y_R(self,u,v,r,δ,n_p):
        return -(1+self.a_H)*self.F_N(u,v,r,δ,n_p)*np.cos(δ)

    def N_R(self,u,v,r,δ,n_p):
        return -(self.x_R+self.a_H*self.x_H)*self.F_N(u,v,r,δ,n_p)*np.cos(δ)

    def F_N(self,u,v,r,δ,n_p):
        u_p=self.ϵ*(1-self.w_P0)*u
        u_R=u_p*np.sqrt(self.η*(1+self.κ*np.sqrt(1+8*self.K_T(u,n_p)/(np.pi*self.J(u,n_p)**2))-1)**2+(1-self.η))
        U=np.sqrt(u**2+v**2)
        β=np.arctan(-v/U)
        r_dash=r*self.L_pp/U
        v_R=U*self.γ_R_plus*(β-self.l_r_dash*r_dash)
        α_R=δ-np.arctan(v_R/u_R)
        U_R=np.sqrt(u_R**2+v_R**2)
        return 0.5*self.ρ*self.A_R*U_R**2*self.f_α*np.sin(α_R)

    def X_P(self,u,v,r,δ,n_p):
        return (1-self.t_P)*self.T_P(u,n_p)

    def K_T(self,u,n_p):
        return self.k_0+self.k_1*self.J(u,n_p)+self.k_2*self.J(u,n_p)**2

    def J(self,u,n_p):
        return u*(1-self.w_P0)/(n_p*self.D_p)

    def T_P(self,u,n_p):
        return self.K_T(u,n_p)*self.ρ*n_p**2*self.D_p**4


@dataclasses.dataclass
class BasicParams:
    ρ: float  # 海水密度
    L_pp: float  # 船長Lpp[m]
    B:float  # 船幅[m]
    d:float  # 喫水[m]
    nabla:float  # 排水量[m^3]
    x_G_dash:float  # 重心位置[m]
    C_b:float  # 方形係数[-]
    D_p:float  # プロペラ直径[m]
    H_R:float  # 舵高さ[m]
    A_R:float  # 舵断面積[m^2]
    t_P:float  # 推力減少率
    w_P0:float  # 有効伴流率
    m_x_dash:float  # 付加質量x(無次元)
    m_y_dash: float  # 付加質量y(無次元)
    J_z_dash: float  # 付加質量Izz(無次元)
    t_R: float  # 操縦抵抗減少率
    x_R_dash: float  # 舵の相対位置
    a_H: float  # 舵力増加係数
    x_H_dash:float  # 舵力増分作用位置
    γ_R_minus: float  # 整流係数
    γ_R_plus: float  # 整流係数
    l_r_dash: float  # 船長に対する舵位置
    x_P_dash: float  # 船長に対するプロペラ位置
    ϵ: float  # プロペラ・舵位置伴流係数比
    κ: float  # 修正係数
    f_α: float  # 直圧力勾配係数
    m: float
    I_zG: float
    η: float
    m_x: float
    m_y: float
    J_z: float
    x_H: float
    x_R: float
    x_G: float

@dataclasses.dataclass
class MPCParams:
    n_horizon: int
    t_step: int
    n_robust: int
    store_full_solution: bool
    δ_set: float
    n_p_set: float
    δ_max: float
    δ_min: float
    n_p_max: float
    n_p_min: float
    u_max: float
    u_min: float
    v_max: float
    v_min: float
    control_duration: int
    model_type: str

@dataclasses.dataclass
class MMGManeuveringParams:
    k_0: float
    k_1: float
    k_2: float
    R_0_dash: float
    X_vv_dash: float
    X_vr_dash: float
    X_rr_dash: float
    X_vvvv_dash: float
    Y_v_dash: float
    Y_r_dash: float
    Y_vvv_dash: float
    Y_vvr_dash: float
    Y_vrr_dash: float
    Y_rrr_dash: float
    N_v_dash: float
    N_r_dash: float
    N_vvv_dash: float
    N_vvr_dash: float
    N_vrr_dash: float
    N_rrr_dash: float
    R_0: float

def set_basic_params(ρ, L_pp, B, d, nabla, x_G_dash, C_b, D_p, H_R, A_R, t_P, w_P0, 
    m_x_dash, m_y_dash, J_z_dash, t_R, x_R_dash, a_H, x_H_dash,
    γ_R_minus, γ_R_plus, l_r_dash, x_P_dash, ϵ, κ, f_α):
    return BasicParams(ρ=ρ,
                        L_pp=L_pp, 
                        B=B, 
                        d=d, 
                        nabla=nabla, 
                        x_G_dash=x_G_dash,
                        C_b=C_b, 
                        D_p=D_p, 
                        H_R=H_R, 
                        A_R=A_R, 
                        t_P=t_P, 
                        w_P0=w_P0, 
                        m_x_dash=m_x_dash, 
                        m_y_dash=m_y_dash, 
                        J_z_dash=J_z_dash, 
                        t_R=t_R, 
                        x_R_dash=x_R_dash, 
                        a_H=a_H, 
                        x_H_dash=x_H_dash, 
                        γ_R_minus=γ_R_minus, 
                        γ_R_plus=γ_R_plus, 
                        l_r_dash=l_r_dash, 
                        x_P_dash=x_P_dash, 
                        ϵ=ϵ, 
                        κ=κ, 
                        f_α=f_α,
                        m=ρ * nabla,
                        I_zG=ρ * nabla * ((0.25 * L_pp) ** 2),
                        η=D_p / H_R,
                        m_x=(0.5 * ρ * (L_pp ** 2) * d) * m_x_dash,
                        m_y=(0.5 * ρ * (L_pp ** 2) * d) * m_y_dash,
                        J_z=(0.5 * ρ * (L_pp ** 4) * d) * J_z_dash,
                        x_H=x_H_dash * L_pp,
                        x_R=x_R_dash*L_pp,
                        x_G=x_G_dash*L_pp
    )

def set_mpc_params(n_horizon, t_step, n_robust, store_full_solution, 
    δ_set, n_p_set, δ_max, δ_min, n_p_max, n_p_min, 
    u_max, u_min, v_max, v_min, control_duration,model_type):
    return MPCParams(
        n_horizon=n_horizon,
        t_step=t_step,
        n_robust= n_robust,
        store_full_solution=store_full_solution,
        δ_set=δ_set,
        n_p_set=n_p_set,
        δ_max=δ_max,
        δ_min=δ_min,
        n_p_max=n_p_max,
        n_p_min=n_p_min,
        u_max=u_max,
        u_min=u_min,
        v_max=v_max,
        v_min=v_min,
        control_duration=control_duration,
        model_type=model_type
        )

def set_mmg_params(k_0, k_1, k_2, 
    R_0_dash, X_vv_dash, X_vr_dash, X_rr_dash, X_vvvv_dash, 
    Y_v_dash, Y_r_dash, Y_vvv_dash, Y_vvr_dash, Y_vrr_dash, Y_rrr_dash, 
    N_v_dash, N_r_dash, N_vvv_dash, N_vvr_dash, N_vrr_dash, N_rrr_dash,
    basic_params
    ):
    return MMGManeuveringParams(
        k_0=k_0, 
        k_1=k_1, 
        k_2=k_2, 
        R_0_dash=R_0_dash, 
        X_vv_dash=X_vv_dash,
        X_vr_dash=X_vr_dash,
        X_rr_dash=X_rr_dash,
        X_vvvv_dash=X_vvvv_dash,
        Y_v_dash=Y_v_dash,
        Y_r_dash=Y_r_dash,
        Y_vvv_dash=Y_vvv_dash,
        Y_vvr_dash=Y_vvr_dash,
        Y_vrr_dash=Y_vrr_dash,
        Y_rrr_dash=Y_rrr_dash,
        N_v_dash=N_v_dash,
        N_r_dash=N_r_dash,
        N_vvv_dash=N_vvv_dash,
        N_vvr_dash=N_vvr_dash,
        N_vrr_dash=N_vrr_dash,
        N_rrr_dash=N_rrr_dash,
        R_0=0.5*basic_params.ρ*(basic_params.L_pp**2)*basic_params.d*R_0_dash
        )

def get_KVLCC2_L7model_basic_params(
    ρ = 1025.0,  # 海水密度
    L_pp = 7.00,  # 船長Lpp[m]
    B = 1.27,  # 船幅[m]
    d = 0.46,  # 喫水[m]
    nabla = 3.27,  # 排水量[m^3]
    x_G = 0.25,  # 重心位置[m]
    C_b = 0.810,  # 方形係数[-]
    D_p = 0.216,  # プロペラ直径[m]
    H_R = 0.345,  # 舵高さ[m]
    A_R = 0.0539,  # 舵断面積[m^2]
    t_P = 0.220,  # 推力減少率
    w_P0 = 0.40,  # 有効伴流率
    m_x_dash = 0.022,  # 付加質量x(無次元)
    m_y_dash = 0.223,  # 付加質量y(無次元)
    J_z_dash = 0.011,  # 付加質量Izz(無次元)
    t_R = 0.387,  # 操縦抵抗減少率
    x_R_dash = -0.500,  # 舵の相対位置
    a_H = 0.312,  # 舵力増加係数
    x_H_dash = -0.464,  # 舵力増分作用位置
    γ_R_minus = 0.395,  # 整流係数
    γ_R_plus = 0.640,  # 整流係数
    l_r_dash = -0.710,  # 船長に対する舵位置
    x_P_dash = -0.480,  # 船長に対するプロペラ位置
    ϵ = 1.09,  # プロペラ・舵位置伴流係数比
    κ = 0.50,  # 修正係数
    f_α = 2.747,  # 直圧力勾配係数
    ):
    return set_basic_params(ρ, L_pp, B, d, nabla, x_G, C_b, D_p, H_R, A_R, t_P, w_P0,
                            m_x_dash, m_y_dash, J_z_dash, t_R, x_R_dash, a_H, x_H_dash,
                            γ_R_minus, γ_R_plus, l_r_dash, x_P_dash, ϵ, κ, f_α)

def get_KVLCC2_L7model_mmg_params(
    k_0 = 0.2931,
    k_1 = -0.2753,
    k_2 = -0.1385,
    R_0_dash = 0.022,
    X_vv_dash = -0.040,
    X_vr_dash = 0.002,
    X_rr_dash = 0.011,
    X_vvvv_dash = 0.771,
    Y_v_dash = -0.315,
    Y_r_dash = 0.083,
    Y_vvv_dash = -1.607,
    Y_vvr_dash = 0.379,
    Y_vrr_dash = -0.391,
    Y_rrr_dash = 0.008,
    N_v_dash = -0.137,
    N_r_dash = -0.049,
    N_vvv_dash = -0.030,
    N_vvr_dash = -0.294,
    N_vrr_dash = 0.055,
    N_rrr_dash = -0.013,
    basic_params = get_KVLCC2_L7model_basic_params()
    ):
    return set_mmg_params(k_0, k_1, k_2,
        R_0_dash, X_vv_dash, X_vr_dash, X_rr_dash, X_vvvv_dash,
        Y_v_dash, Y_r_dash, Y_vvv_dash, Y_vvr_dash, Y_vrr_dash, Y_rrr_dash,
        N_v_dash, N_r_dash, N_vvv_dash, N_vvr_dash, N_vrr_dash, N_rrr_dash, basic_params)