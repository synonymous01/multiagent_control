#!/usr/bin/env python
import rospy
import rosbag
import sys
import datetime
import time
import matplotlib.pyplot as plt

if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')

bag_name = 'test.bag'

for topic, msg, t in rosbag.Bag(bag_name).read_messages():
    if topic == '/robot1/odom':
        pass
    elif topic == '/robot1/robot_pose_ekf/odom_combined'