#!/usr/bin/env python
from robomaster_robot import robomaster_robot
from pynput import keyboard
import rospy


rospy.init_node('robomaster_teleop')
robot_number = rospy.get_param('~robot_number')
init_x = rospy.get_param('~initial_x')
init_y = rospy.get_param('~initial_y')
robot1 = robomaster_robot(robot_number[-1], init_x, init_y)

print("press q to stop. WASD movement")

def onpress(key):
    if not hasattr(key, 'char'):
        return True
    if key.char == 'w':
        print('moving forwards')
        robot1.send_velocities(0.4, 0)
    elif key.char == 's':
        print('moving backwards')
        robot1.send_velocities(-0.4, 0)
    elif key.char == 'd':
        print('moving right')
        robot1.send_velocities(0, 0.4)
    elif key.char == 'a':
        print('moving left')
        robot1.send_velocities(0, -0.4)
    elif key.char == 'q':
        robot1.send_velocities(0, 0)
        return False

def onrelease(key):
    robot1.send_velocities(0, 0)

with keyboard.Listener(
    on_press=onpress,
    on_release=onrelease) as listener:
    listener.join()
    

