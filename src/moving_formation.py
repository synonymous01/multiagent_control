#!/usr/bin/env python
import rospy 
import numpy as np
import matplotlib.pyplot as plt
from robomaster_robot import robomaster_robot
import tf2_ros
import tf.transformations
import time

formation = np.array(
    [
        [0, 0],
        [0.75, 0.25*np.sqrt(3)],
        [0, 0.5*np.sqrt(3)],
        [0.25, 0.25*np.sqrt(3)]
    ]
)

goal_locs = np.array(
    [
        [5, 0.45],
        [10, 0.45],
        [15, 0.45],
        [20, 0.45],
        [25, 0.45],
        [30, 0.45],
        [31.5, 0.45]
    ]
)

dt = 0.05

class formations:
    def __init__(self, total_robots):
        rospy.init_node('moving_formation_controller')
        origins = np.array(
            [
                [0.0, 0.0],
                [0.0, 0.45],
                [0.0, 0.9],
                [0.0, 0.45]
            ]
        )
        self.robots = list()
        for i in range(3):
            self.robots.append(robomaster_robot(i), origins[i,0], origins[i, 1])
        self.positions = np.zeros((4, 2))
        self.yaws = np.zeros((4))
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer) 
        self.count = 0
        self.no_of_robots = total_robots
        self.v_max = 0.3
        self.v_min = 0.01
        self.tolerance = 0.08
        self.errors = np.zeros((4, 2))

    def update_dists(self):
        for i in range(self.no_of_robots):
            try:
                trans = self.tfBuffer.lookup_transform(f'robot{i}_odom_combined', 'world', 0)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            self.positions[i, 0] = trans.transform.translation.x
            self.positions[i, 1] = trans.transform.translation.y
            orientation = [0,0,0,0]
            orientation[0] = trans.transform.rotation.x
            orientation[1] = trans.transform.rotation.y
            orientation[2] = trans.transform.rotation.z
            orientation[3] = trans.transform.rotation.w
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation, axes='xyzs')
            self.yaws[i] = yaw
        

    def update_errors(self):
        self.update_dists()
        for i in range(3):
            self.errors[i, :] = [0, 0]
            for j in range(self.no_of_robots):
                if i == j:
                    continue
                self.errors[i, :] = self.errors[i, :] + ((self.positions[i, :] - self.positions[j, :]) - (formation[i, :] - formation[j, :]))
            self.errors[i, :] = -1 * self.errors[i, :]

        #robot 4 is the ghost, it's error will be calculated using go to goal
        self.errors[3, 0] = -5.5*(self.locs[3, 0] - goal_locs[self.count, 0])
        self.errors[3, 1] = -5 * (self.locs[3, 1] - goal_locs[self.count, 1])

    def create_formation(self):
        robots_reached = [False for _ in range(self.no_of_robots)]
        goal_achieved = all(robots_reached)
        self.update_errors()
        while not goal_achieved:
            for i in range(4):
                err = self.errors[i, :]
                if np.linalg.norm(err) > self.tolerance:
                    #linear velocity update
                    vx = np.cos(self.yaws[i]) * err[0] + np.sin(self.yaws[i]) * err[1]

                    if abs(vx) > self.v_max:
                        vx = np.sign(vx) * self.v_max

                    #angular velocity update
                    omega = -np.sin(self.yaws[i]) * err[0] + np.cos(self.yaws[i]) * err[1]
                else:
                    robots_reached[i] = True

                if robots_reached[3] and self.count != 5:
                    self.count += 1
                    self.count = self.count % 6
                    robots_reached[3] = False

                if i != 3:
                    self.robots[i].send_velocities(vx, omega)
                else:
                    #for ghost robot, we simulate it
                    if abs(vx) > 0.4:
                        vx = np.sign(vx) * 0.4
                    self.yaws[i] = self.yaws[i] + dt * omega
                    self.positions[i, 0] = self.positions[i, 0] + dt * vx * np.cos(self.yaws[i])
                    self.positions[i, 1] = self.positions[i, 1] + dt * vx * np.sin(self.yaws[i])
                    print(self.positions[i, :])

        for i in range(3):
            self.robots.send_velocities(0,0)

        print('formation complete')


try:
    rovers = formations(4)
    rovers.create_formation()
except KeyboardInterrupt:
    pass
        