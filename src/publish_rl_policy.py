#!/usr/bin/env python3
# python file to publish the policy output

import rospy
import numpy as np
import os
import sys
from std_msgs.msg import Float32MultiArray
import argparse

class PublichRLPolicy:
    
    # clamp_joints = np.float64([[-0.7, 0.7], [0, 0.83],[0, 0.96], #thumb
    #                             [0, 0.65], [0, 0.742], #index
    #                             [0, 0.65], [0, 0.742], #middle
    #                             [0, 0.65], [0, 0.742]]) #pinky
    exit_flag = False
    rate = None
    
    def __init__(self, topic='/hand/motors/cmd_joint_angles'):
        self.ros_pub = rospy.Publisher(topic, Float32MultiArray, queue_size=10)
        
        self.joints_arr = np.zeros(9)
        
    def publish_joints(self, policy_joints, start_idx=None, stop_idx=None):
        # publish the policy
        if not start_idx and not stop_idx:
            iter_range = range(policy_joints.shape[1])
        else:
            iter_range = range(start_idx, stop_idx)
        for iter in iter_range:
            msg = Float32MultiArray()
            self.joints_arr = policy_joints[0][iter]
            
            # clamp the joints
            # self.joints_arr = np.clip(self.joints_arr, self.clamp_joints[:,0], self.clamp_joints[:,1])
            
            # convert to degrees becore publishing
            joints_arr_deg = np.rad2deg(self.joints_arr)
            
            msg.data = joints_arr_deg
            
            if not rospy.is_shutdown():
                self.ros_pub.publish(msg)
                
                if (iter%10)==0:
                    rospy.loginfo(f'Publishing policy...{iter}')
                self.rate.sleep()
    
    def set_rate(self, rate=20):
        self.rate = rospy.Rate(rate)

if __name__ == '__main__':
    
    rospy.init_node('publish_rl_policy', anonymous = True)
    
    publish_rl_policy = PublichRLPolicy()
    
    # read arguments from command line
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--direction', help='Rotation direction')
    parser.add_argument('-t', '--task', help='Task name')
    args = parser.parse_args()

    # check the task name
    if args.task:
        task_name = args.task
        rospy.loginfo('Task name: %s', task_name)
    else:
        rospy.logerr('Task name not specified')
        sys.exit()
        
    # check the rotation direction and task
    if task_name=="ball":
        if args.direction:
            rotation_dir = args.direction
            rospy.loginfo('Rotation direction: %s', rotation_dir)
        else:
            rospy.logerr('Rotation direction not specified')
            sys.exit()
    
    # read .npy file 
    root = os.path.dirname(os.path.realpath(__file__))
    if task_name=="ball":
        policy_joints = np.load(root+f'/../rl_recordings/rotate_x_{rotation_dir}_med.npy')
        
        if rotation_dir=="+1":
            start_idx = 225
            stop_idx = 400
        elif rotation_dir=="-1":
            start_idx = 225
            stop_idx = 400
        rate = 20
        publish_rl_policy.set_rate(rate)
        
    elif task_name=="cube":
        policy_joints = np.load(root+'/../rl_recordings/rotate_cube_z_+1.npy')
        # policy_joints = np.load(root+'/../rl_recordings/cube_success_3_reorientation_rot_0.03_dof_poses_2023-12-15_19-34-53_dof_poses.npy')
        start_idx = 0
        stop_idx = 95
        # start_idx = 150
        # stop_idx = 490
        rate = 10
        publish_rl_policy.set_rate(rate)
        
    else:
        sys.exit()
    
    # print the size of the policy
    rospy.loginfo('Policy size: %s', policy_joints.shape)
    
    counter = 0
    
    while not rospy.is_shutdown():
        publish_rl_policy.publish_joints(policy_joints, start_idx, stop_idx)
        counter += 1
        rospy.loginfo(f'Repeating the policy again...{counter}')
        if task_name=="cube":
            break
        