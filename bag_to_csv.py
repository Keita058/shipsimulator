import sqlite3
import pandas as pd
import re
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import os


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

def guidance_to_csv(message, csv_file_name, cols):
    time = [message[i][0]*1e-9 - message[0][0]*1e-9 for i in range(len(message))]
    los_angle = [message[i][1].los_angle for i in range(len(message))]
    message_list = [time, los_angle]
    dic = {cols[i]:message_list[i] for i in range(len(cols))}
    df = pd.DataFrame(dic)
    df.to_csv(csv_file_name)


def main(bag_file_name):
    bag_file = bag_file_name + '/' + bag_file_name + '_0.db3'
    
    vel_cols = ['time', 'u', 'v', 'w', 'roll', 'pitch', 'yaw']
    input_cols = ['time', 'n_p', 'rudder']
    pose_cols = ['time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
    guidance_cols = ['time', 'LOS angle']

    parser = BagFileParser(bag_file)
    topic_names = list(parser.topic_type)

    while topic_names:
        topic_name = topic_names.pop()
        if 'ship' not in topic_name:
            continue
        
        save_dir = bag_file_name + '/' + topic_name.split('/')[1]
        os.makedirs(save_dir, exist_ok = True)

        msg = parser.get_messages(topic_name)
        file_name = save_dir + '/df_' +topic_name.split('/')[2] + '.csv'
        if 'input' in file_name:
            control_to_csv(msg, file_name, input_cols)
        elif 'vel' in file_name:
            twist_to_csv(msg, file_name, vel_cols)
        elif 'pose' in file_name:
            twist_to_csv(msg, file_name, pose_cols)
        elif 'guidance' in file_name:
            guidance_to_csv(msg, file_name, guidance_cols)
        else:
            print("Error :" + topic_name)
            print("This message type is not defined. \n Please check bag_to_csv.py")
        

if __name__ == "__main__":

    bag_file = "subset1"  #読み込みファイル名
    main(bag_file)
