#!/usr/bin/env python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

class robomaster_robot:
    def __init__(self, no):
        self.name = "robot{}".format(no)
        rospy.init_node("{}_controller".format(self.name))
        rospy.Subscriber("{}/robot_pose_ekf/odom_combined".format(self.name), PoseWithCovarianceStamped, self.update_pose)
        self.pub = rospy.Publisher("{}/cmd_vel".format(self.name), Twist, queue_size=10)
        self.position = np.zeros((1, 3))
        self.yaw = float()

    def update_pose(self, data):
        self.position[0] = data.pose.pose.position.x
        self.position[1] = data.pose.pose.position.y
        self.position[2] = data.pose.pose.position.z
        self.yaw, _, __ =  euler_from_quaternion(data.pose.pose.orientation)

    def send_velocities(self, v, omega):
        sending = Twist()
        sending.linear.x = v
        sending.angular.z = omega
        self.pub.publish(sending)
    
    