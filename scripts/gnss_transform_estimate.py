

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

import bag_loader


#load bag
bag_path = "/home/lucas/bags/allmend_transform_estimate.bag"
bag = rosbag.Bag(bag_path)

min_range = 1.0


def main():
    tfs_lio = bag_loader.read_tf_topic(bag, '/kolibri/transform_flu')
    print("Found transforms: ", len(tfs_lio))
    # TODO may reevaluate with /Odometry from LIO directly (transfrom flu comes from mav_state_estimation)

    points_gnss = bag_loader.read_point_topic(bag, '/Gnss')
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
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.align_vectors.html
    rotation, rmsd = R.align_vectors(p_lio, p_gnss)
    print(rotation.as_rotvec())
    print(rotation.as_matrix())
    print(rotation.as_matrix().reshape(-1).tolist())
    
    ## first result with tf (result of pose graph optimization)
    # [[ 0.58120245  0.81369444  0.0102508 ]
    #[-0.81338592  0.58127129 -0.02295701]
    #[-0.02463849  0.00500481  0.9996839 ]]

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

main()
