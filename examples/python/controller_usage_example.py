#!/usr/bin/env python

import rospy
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

class Robot():
    def __init__(self):
        self.dof = 6
        self.robot_goal_pub = rospy.Publisher('/fanuc_stmotion_controller_bringup/robot_goal', Float32MultiArray, queue_size=6)
        self.robot_state_sub = rospy.Subscriber("/fanuc_stmotion_controller_bringup/robot_state", Float32MultiArray, self.robot_state_callback)
        self.robot_travel_time_pub = rospy.Publisher('/fanuc_stmotion_controller_bringup/jpc_travel_time', Float64, queue_size=1)
        self.robot_goal = np.zeros(self.dof)
        self.robot_goal_msg = Float32MultiArray()
        self.robot_state = np.zeros(self.dof)
        self.robot_state_v = np.zeros(self.dof)
        self.robot_state_a = np.zeros(self.dof)
        self.robot_travel_time = Float64()

    def robot_state_callback(self, data):
        for i in range(self.dof):
            self.robot_state[i] = data.data[i*3]
            self.robot_state_v[i] = data.data[i*3 + 1]
            self.robot_state_a[i] = data.data[i*3 + 2]
    
    def set_robot_travel_time(self, t):
        self.robot_travel_time.data = t
        self.robot_travel_time_pub.publish(self.robot_travel_time)

    def drive_robot(self, goal):
        for i in range(self.dof):
            self.robot_goal[i] = goal[i]
        self.robot_goal_msg.data = goal
        self.robot_goal_pub.publish(self.robot_goal_msg)

    def goal_reached(self):
        for i in range(self.dof):
            if(abs(self.robot_state[i] - self.robot_goal[i]) > 0.01 or abs(self.robot_state_v[i]) > 0.001 or abs(self.robot_state_a[i] > 0.001)):
                return False
        return True

def main():
    rospy.init_node('use_controller_node')
    fanuc = Robot()
    goal_list = [[0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 0, -90, 0],
                 [0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 0, -90, 0],
                 [0 ,0, 0, 0, 0, 0]]
    
    goal_idx = -1
    cur_goal = goal_list[0]
    time.sleep(0.1)
    while not rospy.is_shutdown():
        if(goal_idx == -1 or fanuc.goal_reached()):
            print("Reached: ", goal_idx)
            if(goal_idx == -1):
                fanuc.set_robot_travel_time(0.5)
            elif(goal_idx == 1):
                fanuc.set_robot_travel_time(5)
            elif(goal_idx == 3):
                fanuc.set_robot_travel_time(0.1)
            
            goal_idx += 1
            if(goal_idx < len(goal_list)):
                cur_goal = goal_list[goal_idx]   
            else:
                break
            time.sleep(0.01)
        fanuc.drive_robot(cur_goal)
            
if __name__ == '__main__':
    main()