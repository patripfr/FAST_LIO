
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

min_range = 1.0


def main():
    compare_updates()    
    compare_corrupted_bag()
    
def compare_updates():
    paths = ["/home/lucas/bags/gtsam_fusion/original_prediction_update.bag",
             #"/home/lucas/bags/gtsam_fusion/new_prediction_update.bag",
             #"/home/lucas/bags/gtsam_fusion/new_prediction_update2.bag",
             "/home/lucas/bags/gtsam_fusion/new_prediction_update3.bag"
             ]
   
    names = [
             "Original",
             #"Try1",
             #"Try2",
             "Try3"
             ]
    
    vis_odom(paths, names)
def compare_corrupted_bag():
    paths = [
             #"/home/lucas/bags/gtsam_fusion/original_prediction_update.bag",
             "/home/lucas/bags/gtsam_fusion/missing_pcl_no_gps.bag"
             ]
   
    names = [
             #"Original",
             "Corrupted"
             ]
    vis_states(paths, names)
    
def vis_states(bag_paths, names):
    
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(211)# projection='3d')
    ax1 = fig.add_subplot(212)
    
    topics = ['/kolibri/transform_flu',
              '/Gnss',
              '/Odometry']
    
    topic_names = [": Pose Graph",
                 ": GNSS",
                 ": LIO"]
    
    stamp_counter = 0
    for bag_path, name in zip(bag_paths, names):
        bag = rosbag.Bag(bag_path)
        
        for topic, topic_name in zip(topics, topic_names):
            msg_list = bag_loader.read_topic(bag, topic)
            plot_state_estimate_2D(msg_list, ax, name + topic_name)
            plot_time_stamps(msg_list, ax1, stamp_counter, name + topic_name)
            stamp_counter = stamp_counter + 1
             
    ax.legend()
    ax.axis('equal')
    ax1.legend()
    #ax.title("Matching timestamps (but not receipt time!!)")
    plt.show()
    
def vis_odom(paths, names):
    # create plot
    fig1 = plt.figure(figsize=(8, 8))
    fig1_ax1 = plt.subplot2grid(shape=(3, 3), loc=(0, 0), colspan=2, rowspan=2)
    fig1_ax2 = plt.subplot2grid(shape=(3, 3), loc=(0, 2))
    fig1_ax3 = plt.subplot2grid(shape=(3, 3), loc=(1, 2))
    fig1_ax4 = plt.subplot2grid(shape=(3, 3), loc=(2, 2))
    
    fig2 = plt.figure(figsize=(8, 8))
    fig2_ax1 = plt.subplot2grid(shape=(3, 1), loc=(0, 0))
    fig2_ax2 = plt.subplot2grid(shape=(3, 1), loc=(1, 0))
    fig2_ax3 = plt.subplot2grid(shape=(3, 1), loc=(2, 0))
    
    for path, name in zip(paths, names):
        # load bag
        bag = rosbag.Bag(path)
        odoms = bag_loader.read_topic(bag, '/Odometry')
        
        plot_state_estimate_2D(odoms, fig1_ax1, name)
        plot_orientations(odoms, [fig1_ax2, fig1_ax3, fig1_ax4], name)
        plot_state_estimate_1D(odoms, [fig2_ax1, fig2_ax2, fig2_ax3], name)
    
    fig1_ax1.legend()
    plt.show()
        

    
def plot_state_estimate_1D(list_of_containers, ax, label = None):
    translations = np.empty((3,1))
    stamps = list()
    
    for container in list_of_containers:
        translations = np.append(translations, container.t, axis = 1)
        stamps.append(container.stamp)
        
    translations = np.delete(translations, 0, axis = 1)
    
    ax[0].scatter(stamps, translations[0], s= 4, label=label)
    ax[0].set_title("x_transl")
    ax[1].scatter(stamps, translations[1], s= 4, label=label)
    ax[1].set_title("y_transl")
    ax[2].scatter(stamps, translations[2], s= 4, label=label)
    ax[2].set_title("z_transl")
    
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
               
def plot_orientations(container, axes, label = None):
    rpy = [list(), list(), list()]
    stamps = list()
    titles = ["roll", "pitch", "yaw"]
    
    for element in container:
        for angle, sub_list in zip(element.euler, rpy):
            sub_list.append(angle)
        
        stamps.append(element.stamp)
    
    for ax, data, title in zip(axes, rpy, titles):
        ax.scatter(stamps, data, s = 4, label=label)
        ax.legend()
        ax.set_title(title)
    
def plot_time_stamps(list_of_containers, ax, value = 0, label = None):
    times = []
    initial_stamp = list_of_containers[0].stamp
    for container in list_of_containers:
        times.append(container.stamp - initial_stamp)
    ax.scatter(times, [value for t in times], label = label)
    
main()