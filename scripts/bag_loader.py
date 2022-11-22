from datetime import datetime, timedelta
import os
import copy

import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import scipy
from scipy.spatial.transform import Rotation as R
import open3d as o3d


import rosbag
from rospy import Time
import ros_numpy


class TF_Stamped():
    def __init__(self, transl, quat, stamp = None):
        self.stamp = stamp
        self.t = np.asarray([[transl.x, transl.y, transl.z]]).T
        self.quat = quat

        self.rot_matrix = None

        self.generateRotMat()

    def generateRotMat(self):
        self.rot_matrix = R.from_quat([self.quat.x, self.quat.y, self.quat.z, self.quat.w])

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t

def read_tf_topic(bag, topic):
    print('Reading topic: '+ topic)
    data = []

    try:
        msg_type = bag.get_type_and_topic_info()[1][topic][0]
    except KeyError:
        print("Oops!  Topic not found, skipping...")
        msg_type = "not_found_in_bag"

    for topic, msg, t in bag.read_messages(topics=[topic]):
        time = np.datetime64(msg.header.stamp.secs, 's')+np.timedelta64(msg.header.stamp.nsecs, 'ns')

        tf = TF_Stamped(msg.transform.translation, msg.transform.rotation, time)

        data.append(tf)
    return data


class Odom():
    def __init__(self, transl, quat, vel, rot_vel, stamp = None):
        self.stamp = stamp
        self.t = np.asarray([[transl.x, transl.y, transl.z]]).T
        self.quat = quat
        self.vel = vel
        self.rot_vel = rot_vel

        self.rot_matrix = None

        self.generateRotMat()

    def generateRotMat(self):
        self.rot_matrix = R.from_quat([self.quat.x, self.quat.y, self.quat.z, self.quat.w])

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t
        
def read_odom_topic(bag, topic):
    print('Reading topic: '+ topic)
    data = []

    try:
        msg_type = bag.get_type_and_topic_info()[1][topic][0]
    except KeyError:
        print("Oops!  Topic not found, skipping...")
        msg_type = "not_found_in_bag"

    for topic, msg, t in bag.read_messages(topics=[topic]):
        time = np.datetime64(msg.header.stamp.secs, 's')+np.timedelta64(msg.header.stamp.nsecs, 'ns')

        point = Odom(msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular, time)
        data.append(point)

    return data



class Point():
    def __init__(self, transl, stamp = None):
        self.stamp = stamp
        self.p = np.asarray([[transl.x, transl.y, transl.z]]).T
        self.t = self.p
        self.tf = 0

    def generateRotMat(self):
        self.rot_matrix = R.from_quat([self.quat.x, self.quat.y, self.quat.z, self.quat.w])

    def transformPoint(self, point):
        #print(self.rot_matrix.as_matrix(), "\n", self.t)
        return np.dot(self.rot_matrix.as_matrix(), point) + self.t


def read_point_topic(bag, topic):
    print('Reading topic: '+ topic)
    data = []

    try:
        msg_type = bag.get_type_and_topic_info()[1][topic][0]
    except KeyError:
        print("Oops!  Topic not found, skipping...")
        msg_type = "not_found_in_bag"

    for topic, msg, t in bag.read_messages(topics=[topic]):
        time = np.datetime64(msg.header.stamp.secs, 's')+np.timedelta64(msg.header.stamp.nsecs, 'ns')

        point = Point(msg.point, time)

        data.append(point)
    return data
