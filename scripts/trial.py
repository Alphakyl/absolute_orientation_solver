#!/usr/bin/python2
"""
Leica UI node to monitor the prism data
"""

from __future__ import division
from __future__ import print_function
from future import standard_library 
standard_library.install_aliases()
from builtins import str
from past.utils import old_div
from builtins import object
import GeoCom
import actionlib
from std_srvs.srv import *
from optparse import OptionParser
import math
from math import sin,cos
from queue import *
from threading import *
import pdb

import sys
import rospy 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
# 
import math
import tf
import numpy as np 
from numpy import linalg as LA
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
import itertools as it
import time
# from QLabeledValue import *
from leica_ros.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

import message_filters
from leica_ros.srv import *
# import sys 
# sys.path.insert(1, '/home/arpg/catkin_ws/src/leica_ros/scripts')
# from leica_node import *
# from leica_teleop import *

# from std_msgs.srv import *

from ltf.srv import gettf
from ltf.srv import gettfRequest
from ltf.srv import gettfResponse


def mtf(trans1, rot1, trans2, rot2):

    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat   = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return trans3, rot3

def multiply_tfs(trans1, rot1, trans2, rot2,mmn):
    

    trans1_mat = tf.transformations.translation_matrix(trans1)    
    rot1_mat   = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)    
    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)   

    m = np.dot(mat1,mmn)
    mat3 = np.dot(m,mat2)
    
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return trans3, rot3
    
class prism_tf:
    def __init__(self):
        ang = np.array([0,0,0,1])
        pos = np.array([0,0,0])
P1 = prism_tf()
P2 = prism_tf()
P3 = prism_tf()
P4 = prism_tf()
P5 = prism_tf()
P6 = prism_tf()
world = [prism_tf() for _ in range(7)]
w = []
q = []
world_origin_pos = []
world_origin_rot = []
count =1 
c=1
# X = (4,4)
# X = np.zeros(X)
V1 = []
V2 = []
V3 = []
V4 = []
min1 = []
min2 = []

# def cost_fun(x,T1,T2):
#     global min1,V1,V2
    
#     time = rospy.Time(0)
#     x = np.reshape(x,(4,4))
#     m = x
#     Y = V1-np.dot(m,V2)
#     Y = np.hstack([Y.flatten()])
#     min1 = x
#     return LA.norm(Y)

def cost_fun1(x,T1,T2):
    global min1,V1,V2

    tv_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
    # rpy = tf.transformations.quaternion_from_euler(x[3], x[4], x[5])
    # lm   = tf.transformations.quaternion_matrix(rpy)
    # mx = np.dot(tv_mat, lm)

    Y = V1-np.dot(tv_mat,V2)   
    Y = np.hstack([Y.flatten()])

    return LA.norm(Y)


def cost_fun2(x,T1,T2):
    global min1,V1,V2

    # tv_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
    rpy = tf.transformations.quaternion_from_euler(x[3], x[4], x[5])
    lm   = tf.transformations.quaternion_matrix(rpy)
    # mx = np.dot(tv_mat, lm)

    Y = V1-np.dot(lm,V2)   
    Y = np.hstack([Y.flatten()])

    return LA.norm(Y)

# def cost_fun(x,T1,T2):
#     global min1,V1,V2

#     tv_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
#     rpy = tf.transformations.quaternion_from_euler(x[3], x[4], x[5])
#     lm   = tf.transformations.quaternion_matrix(rpy)
#     mx = np.dot(tv_mat, lm)

#     Y = V1-np.dot(mx,V2)   
#     Y = np.hstack([Y.flatten()])

#     return LA.norm(Y)

def cost_func(y,T3,T4):
    global min2,V3,V4

    tv_mat = tf.transformations.translation_matrix(np.array([y[0],y[1],y[2]]))
    rpy = tf.transformations.quaternion_from_euler(y[3], y[4], y[5])
    lm   = tf.transformations.quaternion_matrix(rpy)
    mx = np.dot(tv_mat, lm)

    Y = V3-np.dot(mx,V4)   
    Y = np.hstack([Y.flatten()])
 
    return LA.norm(Y)

class PrismMonitorWidget(QWidget): 
    commandUpdate = pyqtSignal(Transform)
    Current_P = prism_tf()
    counter = 0

    def __init__(self,parent = None):
        self.counter = 0
        self.Current_P = prism_tf()

        super(PrismMonitorWidget,self).__init__()
        layout = QVBoxLayout()
        #Prism 1
        self.counter =1
        self.check = True
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Left Lower Gate') 
        self.prismG1 = QPushButton('Calculate')
        self.prismG1.clicked.connect(self.prismG1_onclick) 
        prismLayout.addWidget(self.prismG1)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)

        #Prism 2
        self.counter =2
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Top Gate') 
        self.prismG2 = QPushButton('Calculate') 
        self.prismG2.clicked.connect(self.prismG2_onclick) 
        prismLayout.addWidget(self.prismG2)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
        
        #Prism 3
        self.counter =3
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Right Lower Gate') 
        self.prismG3 = QPushButton('Calculate') 
        self.prismG3.clicked.connect(self.prismG3_onclick) 
        prismLayout.addWidget(self.prismG3)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
        
        #Prism 4
        self.counter =4
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Left Robot') 
        self.prismR1 = QPushButton('Calculate') 
        self.prismR1.clicked.connect(self.prismR1_onclick) 
        prismLayout.addWidget(self.prismR1)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
        
        #Prism 5
        self.counter =5
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Top Robot') 
        self.prismR2 = QPushButton('Calculate') 
        self.prismR2.clicked.connect(self.prismR2_onclick) 
        prismLayout.addWidget(self.prismR2)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)

        #Prism 6
        self.counter =6
        prismLayout = QVBoxLayout()
        self.controlGroup = QGroupBox('Prism Right Robot') 
        self.prismR3 = QPushButton('Calculate') 
        self.prismR3.clicked.connect(self.prismR3_onclick) 
        prismLayout.addWidget(self.prismR3)
        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
       
        #Exit Button layout
        self.btnQuit = QPushButton('Exit')
        self.btnQuit.clicked.connect(self.btnQuit_onclick)
        layout.addWidget(self.btnQuit)

        self.setLayout(layout)
    
    def minCal1(self,pos,ang):
        global count 
        global V1,V2 # Stores the Transformation Between Prisms and Gate/Robot Origin to Minimize 
        global min1,min2
        pr1 = []  #To store Translation of Gate Prisms with of Gate Origin to minimize locally
        pr2 = []  #To store Translation of Robot Prism to Robot Prism minimize locally
        #Looks Up the static transform from the world_origin_pos to the
        tf_listener = tf.TransformListener()
        if(count==1):
            tf_listener.waitForTransform("/Gate_Origin", "/PrismLeftLowerGate", rospy.Time(),  rospy.Duration(2))
            (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Gate_Origin","/PrismLeftLowerGate", rospy.Time(0))
            for i in prsm_pose:
                pr1.append(i)
            pr1.append(1)
            pr1 = np.asarray(pr1)
            V1.append(pr1)
        if(count==2):
            tf_listener.waitForTransform("/Gate_Origin", "/PrismTopGate", rospy.Time(),  rospy.Duration(2))
            (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Gate_Origin","/PrismTopGate", rospy.Time(0))
            for i in prsm_pose:
                pr1.append(i)
            pr1.append(1)
            pr1 = np.asarray(pr1)
            V1.append(pr1)
        if(count==3):
            tf_listener.waitForTransform("/Gate_Origin", "/PrismRightLowerGate", rospy.Time(),  rospy.Duration(2))
            (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Gate_Origin","/PrismRightLowerGate", rospy.Time(0))
            for i in prsm_pose:
                pr1.append(i)
            pr1.append(1)
            pr1 = np.asarray(pr1)
            V1.append(pr1)

            if(count==3):
                V1 = np.transpose(V1)
                V2 = np.transpose(V2)
                # rot = (0,0,0)
                # r = tf.transformations.quaternion_from_euler(0,0,0)

                xo = np.array([0,0,0,0.785398,0.785398,0.785398])
                # xo   = tf.transformations.quaternion_matrix(np.array(xo[0],x0[1],x))
                solution = minimize(cost_fun1,xo,method='L-BFGS-B',args=(V1,V2))

                trans1_mat = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
                r = tf.transformations.quaternion_from_euler(solution.x[3],solution.x[4],solution.x[5])
                rot1_mat   = tf.transformations.quaternion_matrix(np.array(r))
                G1 = np.dot(trans1_mat, rot1_mat)
                print("printing trans1_mat")
                print(trans1_mat)
                print("Printing Rot1_mat")
                print(rot1_mat)
                print("printing G1")
                print(G1)

                xo = np.array([0,0,0,0.785398,0.785398,0.785398])
                solution2 = minimize(cost_fun2,xo,method='L-BFGS-B',args=(V1,V2))

                trans2_mat = tf.transformations.translation_matrix(np.array([solution2.x[0],solution2.x[1],solution2.x[2]]))
                r = tf.transformations.quaternion_from_euler(solution2.x[3],solution2.x[4],solution2.x[5])
                rot2_mat   = tf.transformations.quaternion_matrix(np.array(r))
                G2 = np.dot(trans2_mat, rot2_mat)
                print(":Printing trasn2_mat")
                print(trans2_mat)
                print("rot2_mat")
                print(rot2_mat)
                print("Printing G2")
                print(G2)



                t1 = tf.transformations.translation_from_matrix(G1)
                r1 = tf.transformations.quaternion_from_matrix(G1)
                t2 = tf.transformations.translation_from_matrix(G2)
                r2 = tf.transformations.quaternion_from_matrix(G2)
                min1 = mtf(t1,r1,t2,r2)

                print("Printing min1")
                print(min1)
                





                # trans1_mat = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
                # r = tf.transformations.quaternion_from_euler(solution.x[3],solution.x[4],solution.x[5])
                # rot1_mat   = tf.transformations.quaternion_matrix(np.array(r))
                # min1 = np.dot(trans1_mat, rot1_mat)

                ####################################################################################################################

                # xo = np.array([0,0,0,0.785398,0.785398,0.785398])
                # solution = minimize(cost_fun,xo,method='L-BFGS-B',args=(V1,V2))
                # trans1_mat = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
                # r = tf.transformations.quaternion_from_euler(solution.x[3],solution.x[4],solution.x[5])
                # rot1_mat   = tf.transformations.quaternion_matrix(np.array(r))
                # min1 = np.dot(trans1_mat, rot1_mat)
                # method='L-BFGS-B',
                # constraints=(),
                # bounds=((-2,2),(-20,20),(-5,5))
                
                print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                print("Printing Minimization Solution")
                print(solution)
                print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
            
        
            
    def minCal2(self,pos,ang):
        global count 
        global V3,V4 # Stores the Transformation Between Prisms and Gate/Robot Origin to Minimize 
        global min2
        pr2 = []  #To store Translation of Robot Prism to Robot Prism minimize locally
        #Looks Up the static transform from the world_origin_pos to the
        tf_listener = tf.TransformListener()
        if(count==4):
            tf_listener.waitForTransform("/Robot_Origin", "/RobotLeftPrism", rospy.Time(),  rospy.Duration(2))
            (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Robot_Origin","/RobotLeftPrism", rospy.Time(0))
            for i in prsm_pose:
                pr2.append(i)
            pr2.append(1)
            pr2 = np.asarray(pr2)
            V3.append(pr2)
        if(count==5):
            tf_listener.waitForTransform("/Robot_Origin", "/RobotTopPrism", rospy.Time(),  rospy.Duration(2))
            (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Robot_Origin","/RobotTopPrism", rospy.Time(0))
            for i in prsm_pose:
                pr2.append(i)
            pr2.append(1)
            pr2 = np.asarray(pr2)
            V3.append(pr2)
        if(count==6):
            tf_listener.waitForTransform("/Robot_Origin", "/RobotRightPrism", rospy.Time(),  rospy.Duration(2))
            (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Robot_Origin","/RobotRightPrism", rospy.Time(0))
            for i in prsm_pose:
                pr2.append(i)
            pr2.append(1)
            pr2 = np.asarray(pr2)
            V3.append(pr2)

        
            if(count==6):
                V3 = np.transpose(V3)
                print("Printing V3")
                print(V3)
                V4 = np.transpose(V4)
                print("Printing V4")
                print(V4)
            
            # y0 = tf.transformations.quaternion_matrix(rot)
            # t = tf.transformations.translation_from_matrix(y0)
            # rot3 = tf.transformations.quaternion_from_matrix(Y)
            # rpy = tf.transformations.euler_from_quaternion(rot3)
            yo = np.array([0,0,0,0.785398,0.785398,0.785398])
            solution = minimize(cost_func,yo,method='L-BFGS-B',args=(V3,V4))
            trans1_mat = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
            r = tf.transformations.quaternion_from_euler(solution.x[3],solution.x[4],solution.x[5])
            rot1_mat   = tf.transformations.quaternion_matrix(np.array(r))
            min2 = np.dot(trans1_mat, rot1_mat)
            # method='L-BFGS-B',
            # constraints=(),
            # bounds=((-2,2),(-20,20),(-5,5))
            
            print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
            print("Printing Solution")
            print(solution)
            print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
            

            

    def call_listener(self,prsm_pose1,prsm_ang1,prsm_pose2,prsm_ang2,prsm_pose3,prsm_ang3):
        try:
            global count,w,q 
            global world_origin_pos  
            global P1,P2,P3
            global V1,V2 # Stores the Transformation Between Prisms and Gate/Robot Origin to Minimize 
            global min1
            tf_listener = tf.TransformListener()
            
            transform1 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(prsm_pose1), tf.transformations.quaternion_matrix(prsm_ang1))
            inversed_tf1 = tf.transformations.inverse_matrix(transform1)
            print(inversed_tf1)
            trans1 = tf.transformations.translation_from_matrix(inversed_tf1)
            rot1 = tf.transformations.quaternion_from_matrix(inversed_tf1)
            trans_multiplied1, rot_multiplied1 = multiply_tfs(P1.pos,P1.ang, trans1, rot1,min1)
            print(trans_multiplied1)
            w.append(trans_multiplied1)
            q.append(rot_multiplied1)
        
            transform2 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(prsm_pose2), tf.transformations.quaternion_matrix(prsm_ang2))
            inversed_tf2 = tf.transformations.inverse_matrix(transform2)
            trans2 = tf.transformations.translation_from_matrix(inversed_tf2)
            rot2 = tf.transformations.quaternion_from_matrix(inversed_tf2)
            trans_multiplied2, rot_multiplied2 = multiply_tfs(P2.pos,P2.ang, trans2, rot2,min1)
            w.append(trans_multiplied2)
            q.append(rot_multiplied2)

            transform3 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(prsm_pose3), tf.transformations.quaternion_matrix(prsm_ang3))
            inversed_tf3 = tf.transformations.inverse_matrix(transform3)
            trans3 = tf.transformations.translation_from_matrix(inversed_tf3)
            rot3 = tf.transformations.quaternion_from_matrix(inversed_tf3)
            trans_multiplied3, rot_multiplied3 = multiply_tfs(P3.pos,P3.ang, trans3, rot3,min1)
            w.append(trans_multiplied3)
            q.append(rot_multiplied3)

            if(count==3):
                world_origin_pos.append(np.mean(w[0:3],axis=0))
                world_origin_rot.append(np.mean(q[0:3],axis=0))
                print("Leica->World -> Pos")
                print(world_origin_pos)
                print("Leica->World -> Rot")
                print(world_origin_rot)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            detection_counter = 0
            #continue

    def call_listener2(self,prsm_pose1,prsm_ang1,prsm_pose2,prsm_ang2,prsm_pose3,prsm_ang3):
        try:
            global count,w,q 
            global world_origin_pos  
            global P4,P5,P6
            global V3,V4 # Stores the Transformation Between Prisms and Gate/Robot Origin to Minimize 
            global min2
            tf_listener = tf.TransformListener()
            
            P4.pos = np.round(P4.pos,4)
            P4.ang = np.round(P4.ang,8)
            P5.pos = np.round(P5.pos,4)
            P5.ang = np.round(P5.ang,8)
            P6.pos = np.round(P6.pos,4)
            P6.ang = np.round(P6.ang,8)

            transform1 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(prsm_pose1), tf.transformations.quaternion_matrix(prsm_ang1))
            inversed_tf1 = tf.transformations.inverse_matrix(transform1)
            trans1 = tf.transformations.translation_from_matrix(inversed_tf1)
            rot1 = tf.transformations.quaternion_from_matrix(inversed_tf1)
            
            trans_multiplied1, rot_multiplied1 = multiply_tfs(np.round(P4.pos,4),np.round(P4.ang,8), trans1, rot1,min2)
            w.append(trans_multiplied1)
            q.append(rot_multiplied1)
        
            transform2 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(prsm_pose2), tf.transformations.quaternion_matrix(prsm_ang2))
            inversed_tf2 = tf.transformations.inverse_matrix(transform2)
            trans2 = tf.transformations.translation_from_matrix(inversed_tf2)
            rot2 = tf.transformations.quaternion_from_matrix(inversed_tf2)
            trans_multiplied2, rot_multiplied2 = multiply_tfs(np.round(P5.pos,4),np.round(P5.ang,8), trans2, rot2,min2)
            w.append(trans_multiplied2)
            q.append(rot_multiplied2)

            transform3 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(prsm_pose3), tf.transformations.quaternion_matrix(prsm_ang3))
            inversed_tf3 = tf.transformations.inverse_matrix(transform3)
            trans3 = tf.transformations.translation_from_matrix(inversed_tf3)
            rot3 = tf.transformations.quaternion_from_matrix(inversed_tf3)
            trans_multiplied3, rot_multiplied3 = multiply_tfs(np.round(P6.pos,4),np.round(P6.ang,8), trans3, rot3,min2)
            w.append(trans_multiplied3)
            q.append(rot_multiplied3)


            if(count==6):

                
                world_origin_pos.append(np.mean(w[3:],axis=0))
                world_origin_rot.append(np.mean(q[3:],axis=0))
                print("Leica->Robot -> Pos")
                print(world_origin_pos[1])
                print("Leica->Robot -> Rot")
                print(world_origin_rot[1])

                
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            detection_counter = 0
            #continue


    def prismG1_onclick(self):
        global V2
        pr = []
        tf_listener = tf.TransformListener()
        #Make the service call to enable the robot
        
        rospy.wait_for_service('prismTransform')
        try: 
            rospy.loginfo('\nPrism:transformations from leica_home are')
            prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
            req = gettfRequest() 
            resp = prismTransform(req)
            #if(self)
            self.Current_P.ang=resp.ang
            self.Current_P.ang=resp.pos 
            global count
            if(count==1):
                P1.ang = resp.ang
                P1.pos = resp.pos
                for i in P1.pos:
                    pr.append(i)
                pr.append(1)
                V2.append(pr)
                self.minCal1(P1.pos,P1.ang)
                print("Prism no.:",count)
                print("*********** world->prism***********")
                print(P1.pos)
                print(P1.ang)
                print("***********************************************")
                
                print('\n')
            if(count<=6):
                count = count +1
            self.prismG1.setEnabled(False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def prismG2_onclick(self):
        global V2
        pr = []
        tf_listener = tf.TransformListener()
        #Make the service call to enable the robot
        rospy.wait_for_service('prismTransform')
        try: 
            # rospy.loginfo('\nPrism:transformations from leica_home are')
            prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
            req = gettfRequest() 
            resp = prismTransform(req)
            #if(self)
            self.Current_P.ang=resp.ang
            self.Current_P.ang=resp.pos 
            global count
            if(count==2):
                P2.ang = resp.ang
                P2.pos = resp.pos
                for i in P2.pos:
                    pr.append(i)
                pr.append(1)
                V2.append(pr)
                self.minCal1(P2.pos,P2.ang)
                print("Prism no.:",count)
                print("*********** world->prism***********")
                print(P2.pos)
                print(P2.ang)
                print("***********************************************")
                print('\n')
            if(count<=6):
                count = count +1
            self.prismG2.setEnabled(False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def prismG3_onclick(self):
        global V2
        pr = []
        tf_listener = tf.TransformListener()
        #Make the service call to enable the robot
        rospy.wait_for_service('prismTransform')
        try: 
            # rospy.loginfo('\nPrism:transformations from leica_home are')
            prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
            req = gettfRequest() 
            resp = prismTransform(req)
            #if(self)
            self.Current_P.ang=resp.ang
            self.Current_P.ang=resp.pos 
            global count
            if(count==3):
                P3.ang = resp.ang
                P3.pos = resp.pos
                for i in P3.pos:
                    pr.append(i)
                pr.append(1)
                V2.append(pr)
                self.minCal1(P3.pos,P3.ang)
                print("Prism no.:",count)
                print("*********** world->prism***********")
                print(P3.pos)
                print(P3.ang)
                print("***********************************************")
                tf_listener.waitForTransform("/Gate_Origin", "/PrismLeftLowerGate", rospy.Time(),  rospy.Duration(2))
                (prsm_pose1, prsm_rot1) = tf_listener.lookupTransform("/Gate_Origin","/PrismLeftLowerGate", rospy.Time(0))
                
                tf_listener.waitForTransform("/Gate_Origin", "/PrismTopGate", rospy.Time(),  rospy.Duration(2))
                (prsm_pose2, prsm_rot2) = tf_listener.lookupTransform("/Gate_Origin","/PrismTopGate", rospy.Time(0))
               
                tf_listener.waitForTransform("/Gate_Origin", "/PrismRightLowerGate", rospy.Time(),  rospy.Duration(2))
                (prsm_pose3, prsm_rot3) = tf_listener.lookupTransform("/Gate_Origin","/PrismRightLowerGate", rospy.Time(0))
                
                self.call_listener(prsm_pose1,prsm_rot1,prsm_pose2,prsm_rot2,prsm_pose3,prsm_rot3)

                print('\n')
                
            if(count<=6):
                count = count +1
            self.prismG3.setEnabled(False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def prismR1_onclick(self):
        global V4
        pr = []
        tf_listener = tf.TransformListener()
        #Make the service call to enable the robot
        rospy.wait_for_service('prismTransform')
        try: 
            # rospy.loginfo('\nPrism:transformations from leica_home are')
            prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
            req = gettfRequest() 
            resp = prismTransform(req)
            #if(self)
            self.Current_P.ang=resp.ang
            self.Current_P.ang=resp.pos 
            global count
            if(count==4):
                P4.ang = resp.ang
                P4.pos = resp.pos
                for i in P4.pos:
                    pr.append(i)
                pr.append(1)
                V4.append(pr)
                self.minCal2(P4.pos,P4.ang)
                print("Prism no.:",count)
                print("***********Prism Position***********")
                print(P4.pos)
                print(P4.ang)
                print("***********************************************")
                
                print('\n')
            if(count<=6):
                count = count +1
            self.prismR1.setEnabled(False)

            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def prismR2_onclick(self):
        
        global V4
        pr = []
        tf_listener = tf.TransformListener()
        #Make the service call to enable the robot
        rospy.wait_for_service('prismTransform')
        try: 
            # rospy.loginfo('\nPrism:transformations from leica_home are')
            prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
            req = gettfRequest() 
            resp = prismTransform(req)
            #if(self)
            self.Current_P.ang=resp.ang
            self.Current_P.ang=resp.pos 
            global count
            if(count==5):
                P5.ang = resp.ang
                P5.pos = resp.pos
                for i in P5.pos:
                    pr.append(i)
                pr.append(1)
                V4.append(pr)
                self.minCal2(P5.pos,P5.ang)
                print("Prism no.:",count)
                print("***********Prism Position***********")
                print(P5.pos)
                print(P5.ang)
                print("***********************************************")
                
                print('\n')
            if(count<=6):
                count = count +1
            self.prismR2.setEnabled(False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def prismR3_onclick(self):
        global V4
        pr = []
        tf_listener = tf.TransformListener()
        #Make the service call to enable the robot
        rospy.wait_for_service('prismTransform')
        try: 
            # rospy.loginfo('\nPrism:transformations from leica_home are')
            prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
            req = gettfRequest() 
            resp = prismTransform(req)
            #if(self)
            self.Current_P.ang=resp.ang
            self.Current_P.ang=resp.pos 
            global count
            if(count==6):
                P6.ang = resp.ang
                P6.pos = resp.pos
                for i in P6.pos:
                    pr.append(i)
                pr.append(1)
                V4.append(pr)
                self.minCal2(P6.pos,P6.ang)
                print("Prism no.:",count)
                print("***********Prism Position***********")
                print(P6.pos)
                print(P6.ang)
                print("***********************************************")
                
                tf_listener.waitForTransform("/Robot_Origin", "/RobotLeftPrism", rospy.Time(),  rospy.Duration(2))
                (prsm_pose4, prsm_rot4) = tf_listener.lookupTransform("/Robot_Origin","/RobotLeftPrism", rospy.Time(0))
            
                tf_listener.waitForTransform("/Robot_Origin", "/RobotTopPrism", rospy.Time(),  rospy.Duration(2))
                (prsm_pose5, prsm_rot5) = tf_listener.lookupTransform("/Robot_Origin","/RobotTopPrism", rospy.Time(0))
                
                tf_listener.waitForTransform("/Robot_Origin", "/RobotRightPrism", rospy.Time(),  rospy.Duration(2))
                (prsm_pose6, prsm_rot6) = tf_listener.lookupTransform("/Robot_Origin","/RobotRightPrism", rospy.Time(0))
                
                self.call_listener2(prsm_pose4,prsm_rot4,prsm_pose5,prsm_rot5,prsm_pose6,prsm_rot6)
                print('\n')
                
            if(count<=6):
                count = count +1
            self.prismR3.setEnabled(False)
            if(count>6):
                print("Prism Count limit reached")
                count = 0        
                self.World_Robot_Origin()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def btnQuit_onclick(self):
        self.parent().close()

    def World_Robot_Origin(self):        
        transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(world_origin_pos[1]), tf.transformations.quaternion_matrix(world_origin_rot[1]))
        inversed_tf = tf.transformations.inverse_matrix(transform)
        trans_inv_1 = tf.transformations.translation_from_matrix(inversed_tf)
        rot_inv_1 = tf.transformations.quaternion_from_matrix(inversed_tf)  
        trans, rot = mtf(world_origin_pos[0], world_origin_rot[0], trans_inv_1, rot_inv_1)
        print("Distance of Origin from ")
        print(trans)
        orientation = tf.transformations.euler_from_quaternion(rot)
        print("Orientation")
        print(orientation)
        tv_mat = tf.transformations.translation_matrix(trans)
        rv_mat   = tf.transformations.quaternion_matrix(rot)
        mat = np.dot(tv_mat, rv_mat)
        
        print("Transformation matrix is")
        print(mat)




def main():

    rospy.init_node('prism_monitor_node')
    app = QApplication(sys.argv)
    mainWidget = PrismMonitorWidget(app)
    mainWindow = QMainWindow()
    mainWindow.setWindowTitle('Prism Position Tracker')
    mainWindow.setCentralWidget(mainWidget)
    mainWindow.setStatusBar(QStatusBar())
    mainWindow.show()
    app.exec_()
    

if __name__ == '__main__':
    try:
	main()
    except rospy.ROSInterruptException:
	pass










