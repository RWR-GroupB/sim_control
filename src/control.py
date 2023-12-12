#!/usr/bin/env python3

import rospy
import mujoco 
import mujoco.viewer
from std_msgs.msg import Float32MultiArray
import numpy as np

class mjSim:
    def __init__(self):

        #subcrbe to the mapped joint angles
        self.ros_sub = rospy.Subscriber('/faive/policy_output', Float32MultiArray, self.callback)


        # setting ros rate
        self.rate = rospy.Rate(20)  # 100 Hz

        #variable fro storing the mapped angles
        self.angles = [0 for i in range(9)]      


        self.m = mujoco.MjModel.from_xml_path('src/sim_control/src/robot-hand.xml')
                        
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

    simulation = mjSim()

    rospy.loginfo_once('Mujoco simulator to camera teleop connected!!!!!!!!!!!!!!')

    try:
        simulation.control()
    except rospy.ROSInterruptException:
        rospy.loginfo('simulator teleop shutdown')
    
