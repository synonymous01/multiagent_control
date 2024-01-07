#!/usr/bin/env python
import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist
from robomaster_robot import robomaster_robot
import numpy as np


follower = robomaster_robot(2, 0, -0.9)

if __name__ == "__main__":
    rospy.init_node('follower_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('robot2_odom_combined', 'robot1_tail', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        omega = -1 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        v = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)


        # if v > 0.2:
        #     v = 0.2 * np.sign(v)
        follower.send_velocities(v, omega)
        # rospy.loginfo("v: {}, omega: {}".format(v, omega))
        rate.sleep()