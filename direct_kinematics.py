#!/usr/bin/env python

import math

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from sympy import *


class Direct_Kinematics():

    def __init__(self):
        rospy.init_node("direct_kinematics")
        rospy.Subscriber("/joint_states",JointState, self.jointStateCallback)
        self.pub = rospy.Publisher("/direct_transform",Pose,queue_size=100)        
     
    def dh_params(self,joint_variable):

        joint_var = joint_variable
        M_PI = math.pi

        # Create DH parameters (data given by maker franka-emika)
        self.dh = [[ 0,      0,        0.333,   joint_var[0]],
            [-M_PI/2,   0,        0,       joint_var[1]],
            [ M_PI/2,   0,        0.316,   joint_var[2]],
            [ M_PI/2,   0.0825,   0,       joint_var[3]],
            [-M_PI/2,  -0.0825,   0.384,   joint_var[4]],
            [ M_PI/2,   0,        0,       joint_var[5]],
            [ M_PI/2,   0.088,    0.107,   joint_var[6]]]
        
        return self.dh
      
    def TF_matrix(self,i,dh):
        # Define Transformation matrix based on DH params
        alpha = dh[i][0]
        a = dh[i][1]
        d = dh[i][2]
        q = dh[i][3]
        
        TF = Matrix([[cos(q),-sin(q), 0, a],
                    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [   0,  0,  0,  1]])
        return TF

    def jointStateCallback(self,message):
        rate = rospy.Rate(100)  # Set rate for the node execution
        self.joint_var = []
        for i in range(0,7):
            self.joint_var.append((message.position[i]))

        dh_parameters = self.dh_params(self.joint_var)

        T_01 = self.TF_matrix(0,dh_parameters)
        T_12 = self.TF_matrix(1,dh_parameters)
        T_23 = self.TF_matrix(2,dh_parameters)
        T_34 = self.TF_matrix(3,dh_parameters)
        T_45 = self.TF_matrix(4,dh_parameters)
        T_56 = self.TF_matrix(5,dh_parameters)
        T_67 = self.TF_matrix(6,dh_parameters)

        T_07 = T_01*T_12*T_23*T_34*T_45*T_56*T_67 
       
        quaternions = tf.transformations.quaternion_from_matrix(T_07)
        translations = tf.transformations.translation_from_matrix(T_07)

        # Writing data to the Pose message for publishing
        panda_pose = Pose()
        panda_pose.position.x = translations[0]
        panda_pose.position.y = translations[1]
        panda_pose.position.z = translations[2]
        panda_pose.orientation.x = quaternions[0]
        panda_pose.orientation.y = quaternions[1]
        panda_pose.orientation.z = quaternions[2]
        panda_pose.orientation.w = quaternions[3]
    
        self.pub.publish(panda_pose)

        rate.sleep()
        
if __name__ == '__main__':
    Direct_Kinematics()
    rospy.spin()
