#!/usr/bin/env python3

import rospy
import mujoco 
import mujoco.viewer
from std_msgs.msg import Float32MultiArray
import numpy as np
import os
import sys
import argparse

class mjSim:
    def __init__(self, topic = '', x_config=None):

        #subcribe to the mapped joint angles
        self.ros_sub = rospy.Subscriber(topic, Float32MultiArray, self.callback)

        # setting ros rate
        self.rate = rospy.Rate(1000)

        #variable for storing the mapped angles
        self.angles = [0 for i in range(9)]      


        # get python file root 
        self.root = os.path.dirname(os.path.realpath(__file__))
        if x_config==2:
            self.m = mujoco.MjModel.from_xml_path(self.root+'/models/scene-cube.xml')
        else:
            self.m = mujoco.MjModel.from_xml_path(self.root+'/models/scene-ball.xml')
                        
        self.d = mujoco.MjData(self.m)  
        
        # rotate the body named "hand_palm" to a quarternion orientation
        if x_config:
            # quarternion from euler angles
            if x_config == 1:
                self.m.body('hand_palm').quat = [ -0.7010574, 0.092296, 0.7010574, -0.092296 ] # -15,-90,180
            elif x_config ==2:
                self.m.body('hand_palm').quat = [ -0.7071068, 0, 0.7071068, 0 ] # 0,-90,180
    
    def callback(self, msg):

        for i in range(9):
            self.angles[i] = np.deg2rad(msg.data[i])
        # self.angles = msg.data

    def control(self):
        with mujoco.viewer.launch_passive(self.m,self.d)  as viewer:
            
            # Set the camera position
            viewer.cam.lookat[0] = -0.00722776 # x-coordinate
            viewer.cam.lookat[1] = 0.01309284  # y-coordinate
            viewer.cam.lookat[2] = 1.03614345  # z-coordinate

            viewer.cam.distance = 0.59
            viewer.cam.elevation = -30.61  # in degrees
            viewer.cam.azimuth = -129.94  # in degrees
            
            while not rospy.is_shutdown():

                self.d.ctrl = self.angles
                mujoco.mj_step(self.m, self.d)

                # Example modification of a viewer option: toggle contact points every two seconds.
                # with viewer.lock():
                #     viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.d.time % 2)

                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                viewer.sync()
                
                # log camera position and orientation
                # rospy.loginfo(f"camera position: {viewer.cam.lookat}")
                # rospy.loginfo(f"camera orientation: {viewer.cam.elevation}")
                # rospy.loginfo(f"camera azimuth: {viewer.cam.azimuth}")
                # rospy.loginfo(f"camera distance: {viewer.cam.distance}")
                

                self.rate.sleep()

    
if __name__ == '__main__':
    
    rospy.init_node('mujoco_rl_sim', anonymous = True)

    TOPIC = '/hand/motors/cmd_joint_angles'
    
    # add argument to get x angle from command line
    parser = argparse.ArgumentParser()
    parser.add_argument('-x', '--x_config', help='x-axis configuration')
    args = parser.parse_args()
    
    # use ros args
    if args.x_config:
        x_config = int(args.x_config)
        rospy.loginfo('x config: %s', x_config)
    else:
        x_config = None
    
    
    simulation = mjSim(TOPIC, x_config)

    rospy.loginfo_once('Opening mujoco simulator!!!')

    try:
        simulation.control()
    except rospy.ROSInterruptException:
        rospy.loginfo('simulator shutdown')
    
