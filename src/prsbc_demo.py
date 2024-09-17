#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16


goal_sectors = [5, 40, 61, 35]
publishers = list()

rospy.init_node('prsbc_demo_goal_publisher')

for i in range(4):
    publishers.append(rospy.Publisher('robot{}/goal_sector'.format(i), Int16, queue_size=1))

while not rospy.is_shutdown():
    for i in range(4):
        sending = Int16()
        sending.data = goal_sectors[i]
        publishers[i].publish(sending)

