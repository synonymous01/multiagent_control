#!/usr/bin/env python
from robomaster_robot import robomaster_robot
from pynput import keyboard

robot1 = robomaster_robot(1)

print("press p to stop. WASD movement")

def onpress(key):
    if not hasattr(key, 'char'):
        return True
    if key.char == 'w':
        print('moving forwards')
        robot1.send_velocities(0.2, 0)
    elif key.char == 's':
        print('moving backwards')
        robot1.send_velocities(-0.2, 0)
    elif key.char == 'a':
        print('moving right')
        robot1.send_velocities(0, 1)
    elif key.char == 'd':
        print('moving left')
        robot1.send_velocities(0, -1)
    elif key.char == 'p':
        robot1.send_velocities(0, 0)
        return False

def onrelease(key):
    robot1.send_velocities(0, 0)

with keyboard.Listener(
    on_press=onpress,
    on_release=onrelease) as listener:
    listener.join()
    

