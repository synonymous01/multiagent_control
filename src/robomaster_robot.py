#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TransformStamped
import tf2_ros
from tf.transformations import quaternion_multiply, quaternion_from_euler, quaternion_inverse, euler_from_quaternion

class robomaster_robot:
    def __init__(self, no, init_x = 0, init_y = 0):
        self.name = "robot{}".format(no)
        # rospy.init_node("{}_controller".format(self.name))
        rospy.Subscriber("{}/robot_pose_ekf/odom_combined".format(self.name), PoseWithCovarianceStamped, self.update_pose)
        self.pub = rospy.Publisher("{}/cmd_vel".format(self.name), Twist, queue_size=10)
        self.init_x = init_x
        self.init_y = init_y

    def update_pose(self, data):
        fixed_initial_rotation = quaternion_from_euler(0, 0, (-np.pi / 2), 'rxyz')
        curr_orientation = data.pose.pose.orientation
        in_quaternions = [curr_orientation.x, curr_orientation.y, curr_orientation.z, curr_orientation.w]
        resultant = quaternion_multiply(fixed_initial_rotation, in_quaternions)
        broadcaster = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "{}_odom_combined".format(self.name)
        # t.transform.rotation = data.pose.pose.orientation
        t.transform.rotation.x = resultant[0]
        t.transform.rotation.y = resultant[1]
        t.transform.rotation.z = resultant[2]
        t.transform.rotation.w = resultant[3]

        _, __, yaw = euler_from_quaternion(resultant, 'rxyz')
        t.transform.translation.x = data.pose.pose.position.x * np.cos(yaw) - data.pose.pose.position.y * np.sin(yaw) + self.init_x
        t.transform.translation.y = data.pose.pose.position.x * np.sin(yaw) + data.pose.pose.position.y * np.cos(yaw) + self.init_y
        t.transform.translation.z = 0.0
        broadcaster.sendTransform(t)
        # self.yaw, _, __ =  euler_from_quaternion(data.pose.pose.orientation)

    def send_velocities(self, vx, vy, omega=0):
        sending = Twist()
        sending.linear.x = vx
        sending.linear.y = vy
        sending.angular.z = omega
        self.pub.publish(sending)
    
    
