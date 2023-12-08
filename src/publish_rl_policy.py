#!/usr/bin/env python3
# python file to publish the policy output

import rospy
import numpy as np
import os
import sys
from std_msgs.msg import Float32MultiArray

class PublichRLPolicy:
    
    clamp_joints = np.float64([[-0.7, 0.7], [0, 0.83],[0, 0.96], #thumb
                                [0, 0.65], [0, 0.742], #index
                                [0, 0.65], [0, 0.742], #middle
                                [0, 0.65], [0, 0.742]]) #pinky
    exit_flag = False
    
    def __init__(self, topic='/hand/motors/cmd_joint_angles'):
        self.ros_pub = rospy.Publisher(topic, Float32MultiArray, queue_size=10)
        
        self.joints_arr = np.zeros(9)
        
        # setting ros rate
        self.rate = rospy.Rate(1)
        
    def publish_joints(self, policy_joints):
        # publish the policy
        for iter in range(policy_joints.shape[1]):
            msg = Float32MultiArray()
            self.joints_arr = policy_joints[1][iter]
            
            # clamp the joints
            self.joints_arr = np.clip(self.joints_arr, self.clamp_joints[:,0], self.clamp_joints[:,1])
            
            # msg.data = self.joints_arr
            # convert to degrees becore publishing
            joints_arr_deg = np.rad2deg(self.joints_arr)
            msg.data = joints_arr_deg
            
            if not rospy.is_shutdown():
                self.ros_pub.publish(msg)
                self.rate.sleep()
        

if __name__ == '__main__':
    
    rospy.init_node('publish_rl_policy', anonymous = True)
    
    publish_rl_policy = PublichRLPolicy()
    
    # reap .npy file 
    root = os.path.dirname(os.path.realpath(__file__))
    policy_joints = np.load(root+'/../rl_recordings/2023-12-08_10-06-58_dof_poses.npy')
    
    # print the size of the policy
    rospy.loginfo('Policy size: %s', policy_joints.shape)
    
    publish_rl_policy.publish_joints(policy_joints)