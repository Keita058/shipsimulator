import sqlite3
import pandas as pd
import re
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import sys
sys.path.append('/home/tabuchi/ros2_ws/src/')
from shipsim_msgs_module.msg import Control


class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        #print('topic_data=',topics_data)
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        #print('topic_type=',self.topic_type)
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        #print('topic_id=',self.topic_id)
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}
        #print('topic_msg_message=',self.topic_msg_message)

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):
        #print('topic_name=',topic_name)
        topic_id = self.topic_id[topic_name]
        #print('topic_id=',topic_id)
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]

def twist_to_csv(message, csv_file_name, cols):
    time = [message[i][0]*1e-9 - message[0][0]*1e-9 for i in range(len(message))]
    linear_x = [message[i][1].linear.x for i in range(len(message))]
    linear_y = [message[i][1].linear.y for i in range(len(message))]
    linear_z = [message[i][1].linear.z for i in range(len(message))]
    angular_x = [message[i][1].angular.x for i in range(len(message))]
    angular_y = [message[i][1].angular.y for i in range(len(message))]
    angular_z = [message[i][1].angular.z for i in range(len(message))]
    message_list = [time, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
    dic = {cols[i]:message_list[i] for i in range(len(cols))}
    df = pd.DataFrame(dic)
    df.to_csv(csv_file_name)

def control_to_csv(message, csv_file_name, cols):
    time = [message[i][0]*1e-9 - message[0][0]*1e-9 for i in range(len(message))]
    n_p = [message[i][1].n_p for i in range(len(message))]
    rudder = [message[i][1].rudder_angle_degree for i in range(len(message))]
    message_list = [time, n_p, rudder]
    dic = {cols[i]:message_list[i] for i in range(len(cols))}
    df = pd.DataFrame(dic)
    df.to_csv(csv_file_name)


if __name__ == "__main__":

        bag_file = "subset/subset"  #読み込みファイル名
        bag_file = bag_file + '_0.db3'

        parser = BagFileParser(bag_file)

        vel = parser.get_messages("/ship1/cmd_vel")
        vel_cols = ['time', 'u', 'v', 'w', 'roll', 'pitch', 'yaw']
        twist_to_csv(vel, 'subset/df_cmd_vel.csv', vel_cols)

        con=parser.get_messages("/ship1/control_input")
        control_cols = ['time', 'n_p', 'rudder']
        control_to_csv(con, 'subset/df_control_input.csv', control_cols)

        cmd_con = parser.get_messages("/ship1/cmd_input")
        control_to_csv(cmd_con, 'subset/df_cmd_input.csv', control_cols)

        obs_pose = parser.get_messages("/ship1/obs_pose")

        pose_cols = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        twist_to_csv(obs_pose, 'subset/df_obs_pose.csv', pose_cols)

        obs_vel = parser.get_messages("/ship1/obs_vel")
        twist_to_csv(obs_vel, 'subset/df_obs_vel.csv', vel_cols)

