#!/usr/bin/env python

import rosbag
import sys
import matplotlib.pyplot as plt

if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')

bag_name = 'test.bag'

plt.figure()
for topic, msg, t in rosbag.Bag(bag_name).read_messages():
    if topic == '/robot1/odom':
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        plt.plot(x, y, 'rx')
    elif topic == '/robot1/robot_pose_ekf/odom_combined':
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        plt.plot(x, y, 'b.')

plt.show()