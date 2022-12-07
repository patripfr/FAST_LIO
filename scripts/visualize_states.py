
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


min_range = 1.0


def main():
    vis_odoms()    
    #vis_corrupted_bag()
    
def vis_odoms():
    # previous implementation
    #original_path = "/home/lucas/bags/gtsam_fusion/original_prediction.bag"
    #original_bag = rosbag.Bag(updated_prediction_path)
    
    #points_original = bag_loader.read_odom_topic(original_bag, '/Odometry')
        
    # new predictions
    updated_prediction_path = "/home/lucas/bags/gtsam_fusion/new_prediction_update.bag"
    updated_bag = rosbag.Bag(updated_prediction_path)
    
    points_odom_updated = bag_loader.read_odom_topic(updated_bag, '/Odometry')
    
    # create plot
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111)# projection='3d')
    
    plot_state_estimate_2D(points_odom_updated, ax, "Updated odometry")
    #plot_state_estimate_2D(points_original, ax, "Original odometry")
    
    ax.legend()
    plt.show()
    
    pass
    
def vis_corrupted_bag():
    bag_path = "/home/lucas/bags/gtsam_fusion/missing_pcl_no_gps.bag"
    bag = rosbag.Bag(bag_path)

    tfs_lio = bag_loader.read_tf_topic(bag, '/kolibri/transform_flu')
    print("Found transforms: ", len(tfs_lio))
    # TODO may reevaluate with /Odometry from LIO directly (transfrom flu comes from mav_state_estimation)

    points_gnss = bag_loader.read_point_topic(bag, '/Gnss')
    print("Found points: ", len(points_gnss))
    
    points_odom = bag_loader.read_odom_topic(bag, '/Odometry')
    print("Found odoms: ", len(points_odom))
    

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(211)# projection='3d')
    
    plot_state_estimate_2D(tfs_lio, ax, "Pose Graph")
    plot_state_estimate_2D(points_gnss, ax, "GNSS")
    plot_state_estimate_2D(points_odom, ax, "LIO")
    ax.legend()
    ax.axis('equal')
    
    ax1 = fig.add_subplot(212)
    plot_time_stamps(tfs_lio, ax1, 1, "Pose Graph")
    plot_time_stamps(points_gnss, ax1, 2, "GNSS")
    plot_time_stamps(points_odom, ax1, 3, "LIO")
    ax.legend()
    #ax.title("Matching timestamps (but not receipt time!!)")
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
    
def plot_time_stamps(list_of_containers, ax, value = 0, label = None):
    times = []
    initial_stamp = list_of_containers[0].stamp
    for container in list_of_containers:
        times.append(container.stamp - initial_stamp)
    #print(times[::50])
    ax.scatter(times, [value for t in times], label = label)
        
    
    
    
main()