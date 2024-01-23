#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from math import floor
import tf2_ros
#this code is written for four robots config.
# robot1, robot2, robot3 are defenders
# robot0 is the attacker
n_cols = 4

def coords_to_sector(x_coord, y_coord):
    # sector_number = x + n_cols * y
    x = floor(x_coord)
    y = floor(y_coord) 
    return (x + y * n_cols) + 1

rate = rospy.Rate(10)

rospy.init_node('Xf_publisher')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10)
publisher = rospy.Publisher('sectors', Float32MultiArray, queue_size=1)


sectors = [0 for _ in range(4)]
while not rospy.is_shutdown():
    sending = Float32MultiArray()
    for i in range(4):
        trans = tfBuffer.lookup_transform(f'robot{i}_odom_combined', 'world', rospy.Time())
        sectors[i] = coords_to_sector(trans.transform.translation.x, trans.transform.translation.y)
    sending = sectors
    publisher.publish(sending)
    