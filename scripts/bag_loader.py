from datetime import datetime, timedelta
import os
import copy

import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import scipy
from scipy.spatial.transform import Rotation as R
import subprocess
import open3d as o3d
import yaml


import rosbag
from rospy import Time
import ros_numpy

available_msg_types = ["nav_msgs/Odometry",
                       "geometry_msgs/TransformStamped",
                       "geometry_msgs/PointStamped",
                       ]


class State():
    def __init__(self, transl, stamp = None):
        self.stamp = stamp
        self.t = np.asarray([[transl.x, transl.y, transl.z]]).T
        
        self.v = None
        self.rot_matrix = None
        self.euler = None
        
        self.rot_vel = None
        
    def generateOrientations(self):
        self.rot_matrix = R.from_quat([self.quat.x, self.quat.y, self.quat.z, self.quat.w])
        self.euler = self.rot_matrix.as_euler('zxy', degrees=True)
        
    def reset_stamp(self, stamp):
        self.stamp = self.stamp - stamp

class TF_Stamped(State):
    def __init__(self, transl, quat, stamp = None):
        super().__init__(transl, stamp)
        
        self.quat = quat
        self.generateOrientations()

    def transformPoint(self, point):
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t
        
class Odom(State):
    def __init__(self, transl, quat, vel, rot_vel, stamp = None):
        super().__init__(transl, stamp)
        self.quat = quat
        self.vel = vel
        self.rot_vel = rot_vel

        self.rot_matrix = None
        self.euler = None
        
        self.generateOrientations()

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t

class Point(State):
    # default constructor applied
    def generateOrientations(self):
        raise Exception("Can't generate rot_mat from point")
        pass

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        if(rot_mat is None):
            raise Exception("Date provided can't be in the past")
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t

def read_topic(bag, topic):
    #print('Reading topic: '+ topic)
    data = []

    try:
        msg_type = bag.get_type_and_topic_info()[1][topic][0]
    except KeyError:
        print("Oops!  Topic not found, skipping...")
        msg_type = "not_found_in_bag"
        
    if not msg_type in available_msg_types:
        print("read " + msg_type + " is not implemented yet")
        return None

    for topic, msg, t in bag.read_messages(topics=[topic]):
        time = msg.header.stamp.secs + (10**-9 * msg.header.stamp.nsecs)
                
        if msg_type == "nav_msgs/Odometry":
            element = Odom(msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular, time)
        elif msg_type == "geometry_msgs/TransformStamped":
            element = TF_Stamped(msg.transform.translation, msg.transform.rotation, time)
        elif msg_type == "geometry_msgs/PointStamped":
            element = Point(msg.point, time)
        else:
            print("sth wrong with msg_type")
            break
            

        data.append(element)

    print("For topic '" + topic + "' available msgs: " + str(len(data)))
    return data

def get_start_time(bag_path):
      info_dict = yaml.safe_load(subprocess.Popen(['rosbag', 'info', '--yaml', bag_path], stdout=subprocess.PIPE).communicate()[0])
      duration = info_dict['duration']
      print("Bag_start", info_dict['start'])
      start_time = info_dict['start']
      return start_time
