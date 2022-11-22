
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
bag_path = "/home/lucas/bags/missing_pcl_no_gps.bag"
bag = rosbag.Bag(bag_path)

min_range = 1.0


def main():
    tfs_lio = bag_loader.read_tf_topic(bag, '/kolibri/transform_flu')
    print("Found transforms: ", len(tfs_lio))
    # TODO may reevaluate with /Odometry from LIO directly (transfrom flu comes from mav_state_estimation)

    points_gnss = bag_loader.read_point_topic(bag, '/Gnss')
    print("Found points: ", len(points_gnss))
    
    points_odom = bag_loader.read_odom_topic(bag, '/Odometry')
    print("Found odoms: ", len(points_odom))
    

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111)# projection='3d')
    
    plot_state_estimate_2D(tfs_lio, ax, "Pose Graph")
    plot_state_estimate_2D(points_gnss, ax, "GNSS")
    plot_state_estimate_2D(points_odom, ax, "LIO")
    ax.legend()
    ax.axis('equal')
    plt.show()
    
def plot_state_estimate_2D(list_of_containers, ax, label = None):
    translations = np.empty((3,1))
    
    for container in list_of_containers:
        translations = np.append(translations, container.t, axis = 1)
        
    ax.scatter(translations[0], translations[1], s= 4, label=label)
    
def plot_state_estimate_3D(tfs, ax):
    translations = np.empty((3,1))
    
    for tf in tfs:
        translations = np.append(translations, tf.t, axis = 1);
        
    ax.scatter(translations[0], 
               translations[1],
               translations[2], 
               c='g', s= 4)
    
    
    
    
main()