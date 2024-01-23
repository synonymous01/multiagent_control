#!/usr/bin/env python

import rosbag
import sys
import csv

if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')

bag_name = '/home/ibrahim/10_squares3.bag'

fields = ['time', 'wheel_odom_x', 'wheel_odom_y', 'angular_vel_z', 'linear_accel_x', 'linear_accel_y', 'cmd_vel_vx', 'cmd_vel_omega', 'ekf_odom_x', 'ekf_odom_y']
data = dict()
for topic, msg, t in rosbag.Bag(bag_name).read_messages():
    seconds = t.to_sec()
    if not data.get(seconds, 0):
        data[seconds] = dict()
    if topic == '/robot1/odom':
        data[seconds]['wheel_odom_x'] = msg.pose.pose.position.x
        data[seconds]['wheel_odom_y'] = msg.pose.pose.position.y
    elif topic == '/robot1/robot_pose_ekf/odom_combined':
        data[seconds]['ekf_odom_x'] = msg.pose.pose.position.x
        data[seconds]['ekf_odom_y'] = msg.pose.pose.position.y
    elif topic == 'robot1/imu_data':
        data[seconds]['angular_vel_z'] = msg.angular_velocity.z
        data[seconds]['linear_accel_y'] = msg.linear_acceleration.y
        data[seconds]['linear_accel_x'] = msg.linear_acceleration.x
    elif topic == '/robot1/cmd_vel':
        data[seconds]['cmd_vel_vx'] = msg.linear.x
        data[seconds]['cmd_vel_omega'] = msg.angular.z
    data[seconds]['time'] = seconds

with open('/home/ibrahim/data.csv', 'w') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=fields)
    writer.writeheader()
    mydict = list(data.values())
    writer.writerows(mydict)