#!/usr/bin/env python3
# python file to publish the policy output

import rospy
import numpy as np
import sys
from std_msgs.msg import Float32MultiArray

class PublishJoints:
    
    def __init__(self, topic='/hand/motors/cmd_joint_angles'):
        self.ros_pub = rospy.Publisher(topic, Float32MultiArray, queue_size=10)
        
        self.joints_arr = np.zeros(9)
        
        # setting ros rate
        self.rate = rospy.Rate(0.5)
        
    def publish_joints(self, joint_data):

        msg = Float32MultiArray()
        
        msg.data = joint_data
        
        rospy.loginfo(msg)
        
        
        while not rospy.is_shutdown():
            self.ros_pub.publish(msg)
            self.rate.sleep()
        

if __name__ == '__main__':
    
    rospy.init_node('custom_jointcmds', anonymous = True)
    
    joints = np.zeros(9)
    
    # read args from command line
    joints[8] = sys.argv[1]
    
    publisher = PublishJoints()
    
    publisher.publish_joints(joints)