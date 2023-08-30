using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using ControlMsg = RosMessageTypes.ShipsimMsgs.ControlMsg;
using System;

public class Ship_movement2 : MonoBehaviour
{
    public Rigidbody rb;
    ROSConnection ros;
    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        rb = GetComponent<Rigidbody>();
        ros.Subscribe<TwistMsg>("/ship2/cmd_vel",OnSubscribe);
        ros.RegisterPublisher<TwistMsg>("/ship2/obs_pose");
        ros.RegisterPublisher<TwistMsg>("/ship2/obs_vel");
    }

    // Update is called once per frame
    void Update()
    {
        //移動後の船の位置、姿勢、速度、角速度を取得
        Vector3 ship_pos = rb.transform.position;
        Vector3 ship_ang = rb.transform.eulerAngles;

        Vector3Msg position = new Vector3Msg(0f, 0f, 0f);
        Vector3Msg attitude = new Vector3Msg(0f, 0f, 0f);
        position.x = ship_pos.z;
        position.y = ship_pos.x;
        attitude.z = ship_ang.y;
        TwistMsg obs_pose = new TwistMsg(position, attitude);

        Vector3 ship_vel = rb.velocity;
        Vector3Msg velocity = new Vector3Msg((float)ship_vel.z, (float)ship_vel.x, (float)ship_vel.y);

        Vector3 ship_ang_vel = rb.angularVelocity;
        Vector3Msg angularVelocity = new Vector3Msg((float)ship_ang_vel.z, (float)ship_ang_vel.x, (float)ship_ang_vel.y);

        TwistMsg obs_vel = new TwistMsg(velocity, angularVelocity);

        ros.Publish("/ship2/obs_pose",obs_pose);
        ros.Publish("/ship2/obs_vel",obs_vel);
        Debug.Log($"ship[2] Publish: x={obs_pose.linear.x}, y={obs_pose.linear.y}, psi={obs_pose.angular.z}");
    }
    void OnSubscribe(TwistMsg msg)
    {
        //現在の船の方位を取得
        Vector3 ang = rb.transform.eulerAngles;
        //船から見たX軸Y軸の速度からワールド座標系のX軸Y軸速度へ変換
        Vector3 world_vel = Rotate_local2World((float)msg.linear.x, (float)msg.linear.y, (float)ang.y);
        //船に、このフレームでの速度を与える
        rb.velocity = new Vector3(world_vel.x, world_vel.y, world_vel.z);
        rb.angularVelocity = new Vector3(0f, (float)msg.angular.z, 0f);

        Debug.Log($"ship[2] Subscribe: u={msg.linear.x}, v={msg.linear.y}, r={msg.angular.z}");
    }

    static Vector3 Rotate_local2World(float u, float v, float theta){
        theta = theta*(float)(Math.PI/180f);
        var res_u = u*Math.Cos(theta)-v*Math.Sin(theta);
        var res_v = u*Math.Sin(theta)+v*Math.Cos(theta);
        var res_vel = new Vector3((float)res_v, 0f, (float)res_u);
        return res_vel;
    }
}
