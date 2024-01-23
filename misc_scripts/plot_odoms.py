#!/usr/bin/env python

import rosbag
import sys
import matplotlib.pyplot as plt
from math import sin, cos, radians

if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')

bag_name = '/home/ibrahim/10_squares3.bag'

plt.figure()
position_x = 0
vx = 0
position_y = 0
vy = 0
current_time = 0
prev_time = 0
yaw = 0
for topic, msg, t in rosbag.Bag(bag_name).read_messages():
    prev_time = current_time
    current_time = t.to_sec()
    dt = current_time - prev_time
    if topic == '/robot1/odom':
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # plt.plot(x, y, 'r.', label="odom")
    elif topic == '/robot1/robot_pose_ekf/odom_combined':
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # plt.plot(x, y, 'b.', label="ekf")
    elif topic == '/robot1/imu_data':
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        angular = msg.angular_velocity.z
        # yaw += angular * dt
        # # vx += accel_x * dt
        # # vy += accel_y * dt
        # position_x +=  (accel_x * cos(yaw) - accel_y * sin(yaw)) * (dt ** 2)
        # position_y += (accel_x * sin(yaw) + accel_y * cos(yaw)) * (dt ** 2)
        # print("yaw: {}, x: {}, y: {}, dt: {}".format(yaw, position_x, position_y, dt))

        # plt.plot(position_x, position_y, 'y.', label="imu") 

        plt.plot(t.to_sec(),accel_x, 'b.', label="x accel")
        plt.plot(t.to_sec(),accel_y, 'r.', label = "y accel")
        plt.plot(t.to_sec(),angular, 'y.', label = "angular")
    
plt.legend()
plt.show()