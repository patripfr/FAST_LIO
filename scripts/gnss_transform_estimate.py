

#%matplotlib widget

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


#load bag
bag_path = "/home/lucas/bags/allmend_transform_estimate.bag"
bag = rosbag.Bag(bag_path)

min_range = 1.0


def main():
    tfs_lio = read_tf_topic(bag, '/kolibri/transform_flu')
    print("Found transforms: ", len(tfs_lio))

    points_gnss = read_point_topic(bag, '/Gnss')
    print("Found points: ", len(points_gnss))

    #create stamps for lookup
    tf_stamps = [tf.stamp for tf in tfs_lio]

    p_lio = []
    p_gnss = []
    # find closest stamp
    for point in points_gnss:
        index = find_nearest(point.stamp, tf_stamps)
        point.tf = tfs_lio[index]

        #print("Norm comparison: ", np.linalg.norm(point.p), np.linalg.norm(point.tf.t))
        if(np.linalg.norm(point.p) < min_range):
            continue

        # single shot:
        angle = np.arctan2(point.tf.t[1], point.tf.t[0]) - np.arctan2(point.p[1], point.p[0])
        #print("Rad ", angle, "Grad: ", angle / 3.141 * 180)

        p_lio.append(point.tf.t)
        p_gnss.append(point.p)

    p_lio = np.asarray(p_lio).reshape((-1,3))
    p_gnss = np.asarray(p_gnss).reshape((-1,3))

    # optimize rotation via kabsch algorithm: (scipy)
    rotation, rmsd = R.align_vectors(p_lio, p_gnss)
    print(rotation.as_rotvec())
    print(rotation.as_matrix())
    print(rotation.as_matrix().reshape(-1).tolist())


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





class Point():
    def __init__(self, transl, stamp = None):
        self.stamp = stamp
        self.p = np.asarray([[transl.x, transl.y, transl.z]]).T
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


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

main()
