#!/usr/bin/env python3

import rospy
import mujoco 
import mujoco.viewer
from std_msgs.msg import Float32MultiArray
import numpy as np
import os
import sys

class mjSim:
    def __init__(self, topic = '/faive/policy_output'):

        #subcribe to the mapped joint angles
        self.ros_sub = rospy.Subscriber(topic, Float32MultiArray, self.callback)

        # setting ros rate
        self.rate = rospy.Rate(20)  # 100 Hz

        #variable for storing the mapped angles
        self.angles = [0 for i in range(9)]      

        # get python file root 
        self.root = os.path.dirname(os.path.realpath(__file__))
        
        self.m = mujoco.MjModel.from_xml_path(self.root+'/robot-hand-v4.xml')
                        
        self.d = mujoco.MjData(self.m)  
    
    def callback(self, msg):

        for i in range(9):
            self.angles[i] = np.deg2rad(msg.data[i])
        # self.angles = msg.data

    def control(self):
        with mujoco.viewer.launch_passive(self.m,self.d)  as viewer:
            while not rospy.is_shutdown():

                self.d.ctrl = self.angles
                mujoco.mj_step(self.m, self.d)

                # Example modification of a viewer option: toggle contact points every two seconds.
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.d.time % 2)

                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                viewer.sync()

                self.rate.sleep()

    
if __name__ == '__main__':
    
    rospy.init_node('mujoco_teleop', anonymous = True)
    
    # select which topic to subscribe to using the command line argument
    if len(sys.argv) > 1:
        first_arg = sys.argv[1]
        if (first_arg=='gui'):
            TOPIC = '/hand/motors/cmd_joint_angles'
        elif (first_arg=='teleop'):
            TOPIC = '/faive/policy_output'
        else:
            rospy.loginfo_once("Invalid argument. Please enter either 'gui' or 'teleop', using teleop as default")
            TOPIC = '/faive/policy_output'
        rospy.loginfo_once(f"argument: {sys.argv[1]}")
    else:
        TOPIC = '/faive/policy_output'
    
    

    simulation = mjSim(TOPIC)

    rospy.loginfo_once('Mujoco simulator to camera teleop connected!!!!!!!!!!!!!!')

    try:
        simulation.control()
    except rospy.ROSInterruptException:
        rospy.loginfo('simulator teleop shutdown')
    
