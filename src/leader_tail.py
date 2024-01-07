#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class leader_tail:
    def __init__(self):
        self.pub = rospy.Publisher('/tf', TFMessage, queue_size=1)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t = TransformStamped()
            t.header.frame_id = "robot1_odom_combined"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "robot1_tail"
            t.transform.translation.x = -0.25
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfm = TFMessage([t])
            self.pub.publish(tfm)

            rate.sleep()
            # rospy.spin()

if __name__ == "__main__":
    rospy.init_node('fixed_tail_broadcaster')
    tfb = leader_tail()
    rospy.spin()
        