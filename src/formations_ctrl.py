#!/usr/bin/env python
import numpy as np
import rospy
from robomaster_robot import robomaster_robot
import tf2_ros
import tf.transformations



    
class formations:
    def __init__(self, total_robots):
        self.total_robots = total_robots
        rospy.init_node('formation_controller')
        
        initial_positions = np.array(
            [
                [0, 0],
                [0, 0.9],
                [0, 1.8]
            ]
        )
        
        self.loc_set = np.array(
             [
                  [0, 0],
                  [1, 1],
                  [0, 1]
             ]
        )
        self.robots = list()
        for i in range(total_robots):
            self.robots.append(robomaster_robot(i), initial_positions[i,0], initial_positions[i,1])

        self.errs = np.zeros((self.total_robots, 2), dtype=np.float16)
        self.robots_reached = [False for _ in range(self.total_robots)]
        self.v_max = 0.2
        self.tolerance = 0.3
        self.positions = np.zeros((self.total_robots, 2))
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.yaws = np.zeros((self.total_robots))
        self.update_errors()


    def update_dists(self):
        for i in range(self.total_robots):
            try:
                trans = self.tfBuffer.lookup_transform(f'robot{i}_odom_combined','world', 0)
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
            self.errs[0,:] = -1*(((self.positions[0,:] - self.positions[1,:]) - (self.loc_set[0,:] - self.loc_set[1,:])) + ((self.positions[0,:] - self.positions[2,:]) - (self.loc_set[0,:] - self.loc_set[2,:])))
            self.errs[1,:] = -1*(((self.positions[1,:] - self.positions[0,:]) - (self.loc_set[1,:] - self.loc_set[0,:])) + ((self.positions[1,:] - self.positions[2,:]) - (self.loc_set[1,:] - self.loc_set[2,:])))
            self.errs[2,:] = -1*(((self.positions[2,:] - self.positions[1,:]) - (self.loc_set[2,:] - self.loc_set[1,:])) + ((self.positions[2,:] - self.positions[0,:]) - (self.loc_set[2,:] - self.loc_set[0,:])))
    
    def create_formation(self):
        goal_achieved = all(self.robots_reached)
        self.update_errors()
        vx = 0
        omega = 0
        while not goal_achieved:
            for i in range(self.total_robots):
                err = self.errs[i, :]
                if np.linalg.norm(err) > self.tolerance:
                    vx = np.cos(self.yaws[i]) * err[0] + np.sin(self.yaws[i]) * err[1]

                    if abs(vx) > self.v_max:
                        vx = np.sign(vx) * self.v_max

                    omega = -np.sin(self.yaws[i]) * err[0] + np.cos(self.yaws[i]) * err[1]
                    # omega = omega * (180/np.pi)
                else:
                    self.robots_reached[i] = True
                    print('robot {} reached'.format(i + 1))

                self.robots[i].send_velocities(vx, omega)
            self.update_errors()
            goal_achieved = all(self.robots_reached)

        print('formation complete')


try: 
    triangle = formations(3)
    triangle.create_formation()

except rospy.ROSInterruptException:
    pass