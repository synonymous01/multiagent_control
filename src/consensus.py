#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from robomaster_robot import robomaster_robot
import tf_conversions
import tf2_ros


    
class formations:
    def __init__(self, total_robots):
        self.total_robots = total_robots
        rospy.init_node('formation_controller')
        
        self.loc_set = np.array(
             [
                  [0, 0],
                  [1, 1],
                  [0, 1]
             ]
        )
        self.robots = list()
        for i in range(total_robots):
            self.robots.append(robomaster_robot(i))

        self.errs = np.zeros((self.total_robots, 2), dtype=np.float16)
        self.robots_reached = [False for _ in range(self.total_robots)]
        self.v_max = 0.2
        self.tolerance = 0.3
        self.update_errors()

    def update_errors(self):
            self.errs[0,:] = -1*(((self.robots[0].position - self.robots[1].position) - (self.loc_set[0,:] - self.loc_set[1,:])) + ((self.robots[0].position - self.robots[2].position) - (self.loc_set[0,:] - self.loc_set[2,:])))
            self.errs[1,:] = -1*(((self.robots[1].position - self.robots[0].position) - (self.loc_set[1,:] - self.loc_set[0,:])) + ((self.robots[1].position - self.robots[2].position) - (self.loc_set[1,:] - self.loc_set[2,:])))
            self.errs[2,:] = -1*(((self.robots[2].position - self.robots[1].position) - (self.loc_set[2,:] - self.loc_set[1,:])) + ((self.robots[2].position - self.robots[0].position) - (self.loc_set[2,:] - self.loc_set[0,:])))
    
    def create_formation(self):
        goal_achieved = all(self.robots_reached)
        self.update_errors()
        vx = 0
        omega = 0
        while not goal_achieved:
            for i in range(self.total_robots):
                err = self.errs[i, :]
                if np.linalg.norm(err) > self.tolerance:
                    vx = np.cos(self.robots[i].yaw) * err[0] + np.sin(self.robots[i].yaw) * err[1]

                    if abs(vx) > self.v_max:
                        vx = np.sign(vx) * self.v_max

                    omega = -np.sin(self.robots[i].yaw) * err[0] + np.cos(self.robots[i].yaw) * err[1]
                    omega = omega * (180/np.pi)
                else:
                    self.robots_reached[i] = True
                    print('robot {} reached'.format(i + 1))

                self.robots[i].send_velocities(vx, omega)
            self.update_errors()
            goal_achieved = all(self.robots_reached)


try: 
    triangle = formations(3)
    triangle.create_formation()
except rospy.ROSInterruptException:
    pass