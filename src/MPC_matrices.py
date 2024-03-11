#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from math import floor
import tf2_ros
#this code is written for four robots config.
# robot1, robot2, robot3 are defenders
# robot0 is the attacker
n_cols = 8
meter_per_square_length = 0.5

def coords_to_sector(x_coord, y_coord):
    # sector_number = x + n_cols * y
    x = floor(x_coord/meter_per_square_length)
    y = floor(y_coord/meter_per_square_length) 
    return (x + y * n_cols) + 1


rospy.init_node('Xf_publisher')
# rate = rospy.Rate(10)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10)
publisher = rospy.Publisher('sectors', Int32MultiArray, queue_size=1)


sectors = [49, 1, 3, 5]
while not rospy.is_shutdown():
    sending = Int32MultiArray()
    for i in range(4):
        try:
            trans = tfBuffer.lookup_transform('world', 'robot{}_odom_combined'.format(i), rospy.Time())
        except:
            rate.sleep()
            continue
        # rospy.loginfo("for robot {}, x: {}, y: {}".format(i, trans.transform.translation.x, trans.transform.translation.y))

        sectors[i] = coords_to_sector(trans.transform.translation.x, trans.transform.translation.y)
    rospy.loginfo("sectors sent: {}".format(sectors))
    sending.data = sectors
    publisher.publish(sending)
    