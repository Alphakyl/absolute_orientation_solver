#!/usr/bin/python2
"""
Leica UI node to monitor the prism data

Horn's Formuala : https://www.mathworks.com/matlabcentral/fileexchange/26186-absolute-orientation-horn-s-method
"""

import sys
import rospy 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
# 
import math
import tf
import numpy as np 
import tf2_ros
from numpy import linalg as LA
from numpy.linalg import inv
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
import itertools as it
import time
# from QLabeledValue import *
from leica_ros.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
from geometry_msgs.msg import *

import message_filters

from leica_ros.srv import *

from ltf.srv import gettf
from ltf.srv import gettfRequest
from ltf.srv import gettfResponse

import threading

def normalize_quaternion(rot):
    ratio = math.sqrt(rot[0]**2 + rot[1]**2 + rot[2]**2 + rot[3]**2)
    return (rot[0]/ratio, rot[1]/ratio, rot[2]/ratio, rot[3]/ratio)


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
P1 = prism_tf() #Global Objects representing Prism
P2 = prism_tf()
P3 = prism_tf()
P4 = prism_tf()
P5 = prism_tf()
P6 = prism_tf()
count =1 #Keeps Track of prisms under objective in the UI 
# c=1
V1 = [] # Stores Translation Vectors Vstack for Prisms on the Leica->Gate_Prism
V2 = [] # Stores Translation Vectors Vstack for Prisms on the World->Gate_Prism
V3 = [] # Stores Translation Vectors Vstack for Prisms on the Leica->Robot_Prism
V4 = [] # Stores Translation Vectors Vstack for Prisms on the Robot_Origin->Robot_Prism
min1 = [] # (x,y,z,r,p,y) of Minimization according to Heron'S formula Leica->World
min2 = [] # (x,y,z,r,p,y) of Minimization according to Heron'S formula Leica->Base_link



# def cost_fun(x,T1,T2):
#     global V1,V2

#     tv_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
#     rpy = tf.transformations.quaternion_from_euler(x[3], x[4], x[5])
#     rpn = normalize_quaternion(rpy)
#     lm   = tf.transformations.quaternion_matrix(rpn)
#     mx = np.dot(tv_mat, lm)

#     Y = V1-np.dot(mx,V2)   
#     Y = np.hstack([Y.flatten()])

#     return LA.norm(Y)

def cost_fun(x,T1,T2):
    global V1,V2

    tv_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
    # rpy = np.array([x[3], x[4], x[5],x[6]])
    # print(type(rpy))
    # print(rpy.shape)
    # rpn = normalize_quaternion(np.array([x[3], x[4], x[5],x[6]]))
    lm   = tf.transformations.quaternion_matrix(normalize_quaternion(np.array([x[3], x[4], x[5],x[6]])))
    mx = np.dot(tv_mat, lm)

    Y = V1-np.dot(mx,V2)
    Z = LA.norm(Y[0,:])+LA.norm(Y[1,:])+LA.norm(Y[2,:])   
    # Y = np.hstack([Y.flatten()])

    # return LA.norm(Y)
    return Z


# def cost_func(x,T3,T4):
#     global V3,V4

#     tv_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
#     rpy = tf.transformations.quaternion_from_euler(x[3], x[4], x[5])
#     rpn = normalize_quaternion(rpy)
#     lm   = tf.transformations.quaternion_matrix(rpn)
#     mx = np.dot(tv_mat, lm)
#     Y = V3-np.dot(mx,V4)   
#     Y = np.hstack([Y.flatten()])

#     return LA.norm(Y)

def cost_func(x,T3,T4):
    global V3,V4

    tv_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
    # rpy = tf.transformations.quaternion_from_euler(x[3], x[4], x[5])
    # rpn = normalize_quaternion(rpy)

    lm   = tf.transformations.quaternion_matrix(normalize_quaternion(np.array([x[3], x[4], x[5],x[6]])))
    mx = np.dot(tv_mat, lm)
    # lm   = tf.transformations.quaternion_matrix(rpn)
    # mx = np.dot(tv_mat, lm)
    Y = V3-np.dot(mx,V4)   
    Z = LA.norm(Y[0,:])+LA.norm(Y[1,:])+LA.norm(Y[2,:]) 

    # Y = np.hstack([Y.flatten()])

    # return LA.norm(Y)
    return Z

class PrismMonitorWidget(QWidget): 
    commandUpdate = pyqtSignal(Transform)
    Current_P = prism_tf()
    counter = 0

    availablePrisms = [
        {
            "name" : "big_360",
            "num"  : 3,
            "constant" : 34.4
        },
        {
            "name" : "mini_360",
            "num"  : 7,
            "constant" : 30.0 
        },
        {
            "name" : "mini_lp",
            "num"  : 1,
            "constant" :  17.5
        }
    ]

    def __init__(self,parent = None):
        self.counter = 0
        self.Current_P = prism_tf()

        super(PrismMonitorWidget,self).__init__()
        layout = QVBoxLayout()
        #Prism 1
        self.counter =1
        self.check = True
        prismLayout = QHBoxLayout()
        self.controlGroup = QGroupBox('Prism Left Lower Gate') 

        self.prismG1 = QPushButton('Calculate')
        self.prismG1.clicked.connect(self.prismG1_onclick) 
        prismLayout.addWidget(self.prismG1)

        self.prismG1toggle = QPushButton('Toggle')
        self.prismG1toggle.clicked.connect(self.prismG1toggle_onclick) 
        prismLayout.addWidget(self.prismG1toggle)

        self.prismG1name = "mini_lp"
        self.prismG1namelabel = QLabel(self.prismG1name) 
        prismLayout.addWidget(self.prismG1namelabel)

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

        # stop Leica tracking
        rospy.wait_for_service('leica_node/stop_tracking')
        stop_tracking_svc = rospy.ServiceProxy('leica_node/stop_tracking', SetBool)
        try:
            rospy.loginfo("Stopping tracking")
            stop_tracking_req = SetBoolRequest()
            stop_tracking_req.data = False
            stop_tracking_resp = stop_tracking_svc(stop_tracking_req)
            rospy.loginfo("StopTrackingResponse: %s",stop_tracking_resp.__str__())
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
            return 

        # launch publisher thread
        self.pub_thread = threading.Thread(target=self.pubTF, args=())
        self.pub_thread.start()
    
    def minCal1(self,pos,ang):

        #Looks Up the static transform from the world_origin_pos to the respective Prisms 


        global count 
        global V1,V2 # Stores the Transformation Between Prisms and Gate/Robot Origin to Minimize 
        global min1,min2
        # ml = []
        pr1 = []  #To store Translation of Gate Prisms with of Gate Origin to minimize locally
        pr2 = []  #To store Translation of Robot Prism to Robot Prism minimize locally
        #Looks Up the static transform from the world_origin_pos to the respective Prisms 
        tf_listener = tf.TransformListener()
        if(count==1):
            tf_listener.waitForTransform("/Gate_Origin", "/PrismLeftLowerGate", rospy.Time(),  rospy.Duration(2))
            (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Gate_Origin","/PrismLeftLowerGate", rospy.Time(0))
            for i in prsm_pose:
                pr1.append(i)
            pr1.append(1) # (1*3) -> (1*4) translation vector 
            pr1 = np.asarray(pr1) #List->Np.array
            V1.append(pr1) #Creating Hstack read off of mentioned TF 
        if(count==2):
            tf_listener.waitForTransform("/Gate_Origin", "/PrismTopGate", rospy.Time(),  rospy.Duration(2))
            (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Gate_Origin","/PrismTopGate", rospy.Time(0))
            for i in prsm_pose:
                pr1.append(i) 
            pr1.append(1) #(1*3) -> (1*4) translation vector 
            pr1 = np.asarray(pr1)
            V1.append(pr1) #Creating Hstack read off of mentioned TF (1*4) -> (2*4)
        if(count==3):
            tf_listener.waitForTransform("/Gate_Origin", "/PrismRightLowerGate", rospy.Time(),  rospy.Duration(2))
            (prsm_pose, prsm_rot) = tf_listener.lookupTransform("/Gate_Origin","/PrismRightLowerGate", rospy.Time(0))
            for i in prsm_pose:
                pr1.append(i)
            pr1.append(1)
            pr1 = np.asarray(pr1)
            V1.append(pr1) #Creating Hstack read off of mentioned TF (2*4) -> (3*4)

            if(count==3):
                V1 = np.transpose(V1) 
                V2 = np.transpose(V2)
                

                # xo = np.array([0,0,0,0,0,0])
                # solution = minimize(cost_fun,xo,method='L-BFGS-B',args=(V1,V2))
                # # print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                # # print("Printing Minimization Solution")
                # # print(solution)
                # # print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                # trans1 = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
                # r = tf.transformations.quaternion_from_euler(solution.x[3],solution.x[4],solution.x[5])
                # r = normalize_quaternion(r) #Vital for Correct Solution 
                # rot1   = tf.transformations.quaternion_matrix(np.array(r))
                # min1 = np.dot(trans1,rot1)
                # print("World->Leica")
                # print(min1)





                # xo = np.array([0,0,0,0,0,0,0])
                # solution = minimize(cost_fun,xo,method='L-BFGS-B',args=(V1,V2))
                # # print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                # # print("Printing Minimization Solution")
                # # print(solution)
                # # print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                # trans1 = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
                # # r = tf.transformations.quaternion_from_euler(solution.x[3],solution.x[4],solution.x[5])
                # r = np.array([solution.x[3],solution.x[4],solution.x[5],solution.x[6]])
                # r = normalize_quaternion(r) #Vital for Correct Solution 
                # rot1   = tf.transformations.quaternion_matrix(np.array(r))
                # min1 = np.dot(trans1,rot1)
                # print("World->Leica")
                # print(min1)

                xo = np.array([0,0,0,0,0,0,1])
                solution = minimize(cost_fun,xo,method='L-BFGS-B',args=(V1,V2))
                # print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                # print("Printing Minimization Solution")
                # print(solution)
                # print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                trans1 = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
                # r = tf.transformations.quaternion_from_euler(solution.x[3],solution.x[4],solution.x[5])
                r = np.array([solution.x[3],solution.x[4],solution.x[5],solution.x[6]])
                r = normalize_quaternion(r) #Vital for Correct Solution 
                rot1   = tf.transformations.quaternion_matrix(np.array(r))
                min1 = np.dot(trans1,rot1)
                print("World->Leica")
                print(min1)

       
            
    def minCal2(self,pos,ang):
        
        #Looks Up the static transform from the world_origin_pos to the Respective Prisms 

        global count 
        global V3,V4 # Stores the Transformation Between Prisms and Gate/Robot Origin to Minimize 
        global min2
        pr2 = []  #To store Translation of Robot Prism to Robot Prism minimize locally
        
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

            xo = np.array([0,0,0,0,0,0,1])
            solution = minimize(cost_func,xo,method='L-BFGS-B',args=(V3,V4))
            # print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
            # print("Printing Solution")
            # print(solution)
            # print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
            trans2 = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
            # r = tf.transformations.quaternion_from_euler(solution.x[3],solution.x[4],solution.x[5])
            r = np.array([solution.x[3],solution.x[4],solution.x[5],solution.x[6]])
            r1 = normalize_quaternion(r) #Vital for Correct Solution
            rot2   = tf.transformations.quaternion_matrix(np.array(r1))
            min2 = np.dot(trans2, rot2) 
            
            print("Robot->Leica")
           
            print(min2)

    def prismG1toggle_onclick(self):
        current_idx = next((i for i in range(len(self.availablePrisms)) if self.availablePrisms[i]["name"]==self.prismG1name),None)
        if current_idx is None:
            rospy.logwarn("The current prism name is invalid")
            return
        next_idx = current_idx+1
        if next_idx >= len(self.availablePrisms):
            next_idx = 0
        self.prismG1name = self.availablePrisms[next_idx]["name"]
        self.prismG1namelabel.setText(self.prismG1name)

    def prismG1_onclick(self):


        #Make the service call to enable the robot to get tf data

        global V2
        pr = []
        tf_listener = tf.TransformListener()

        # set prism type in Leica to that which is currently displayed in the GUI
        rospy.wait_for_service('leica_node/set_prism_type')
        set_prism_type_svc = rospy.ServiceProxy('leica_node/set_prism_type', SetPrismType)
        try:
            rospy.loginfo("Setting to prism: %s",self.prismG1name)
            set_prism_type_req = SetPrismTypeRequest()
            set_prism_type_req.name = self.prismG1name
            set_prism_type_resp = set_prism_type_svc(set_prism_type_req)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
            return 

        # start Leica tracking
        rospy.wait_for_service('leica_node/start_tracking')
        start_tracking_svc = rospy.ServiceProxy('leica_node/start_tracking', StartTracking)
        try:
            rospy.loginfo("Starting tracking")
            start_tracking_resp = start_tracking_svc()
            rospy.loginfo("StartTrackingResponse: %s",start_tracking_resp.__str__())
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
            return 

        # get tf between Leica and prism 
        rospy.wait_for_service('prismTransform')
        try: 
            rospy.loginfo('\nPrism:transformations from leica_home are')
            prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
            req = gettfRequest() 
            resp = prismTransform(req)
            #if(self)
            self.Current_P.ang=resp.ang
            self.Current_P.ang=resp.pos 
            # global count
            # if(count==1):
            P1.ang = resp.ang
            P1.pos = resp.pos
            for i in P1.pos:
                pr.append(i)
            pr.append(1)
            V2.append(pr)
            self.minCal1(P1.pos,P1.ang)
            print("Prism no.:",count)
            print("*********** Leica->Left Bottom Gate Prism***********")
            print(P1.pos)
            print(P1.ang)
            print("***********************************************")
            
            print('\n')
            # if(count<=6):
            #     count = count +1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 

        # stop Leica tracking
        rospy.wait_for_service('leica_node/stop_tracking')
        stop_tracking_svc = rospy.ServiceProxy('leica_node/stop_tracking', SetBool)
        try:
            rospy.loginfo("Stopping tracking")
            stop_tracking_req = SetBoolRequest()
            stop_tracking_req.data = False
            stop_tracking_resp = stop_tracking_svc(stop_tracking_req)
            rospy.loginfo("StopTrackingResponse: %s",stop_tracking_resp.__str__())
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
            return 
            
        # disable calculate button    
        self.prismG1.setEnabled(False)

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
            # global count
            # if(count==2):
            P2.ang = resp.ang
            P2.pos = resp.pos
            for i in P2.pos:
                pr.append(i)
            pr.append(1)
            V2.append(pr)
            self.minCal1(P2.pos,P2.ang)
            print("Prism no.:",count)
            print("*********** Leica->Top Gate Prism***********")
            print(P2.pos)
            print(P2.ang)
            print("***********************************************")
            print('\n')
            # if(count<=6):
            #     count = count +1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.prismG2.setEnabled(False)

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
            # global count
            # if(count==3):
            P3.ang = resp.ang
            P3.pos = resp.pos
            print("Prism no.:",count)
            print("*********** Leica->Right Bottom Gate***********")
            print(P3.pos)
            print(P3.ang)
            print("***********************************************")

            for i in P3.pos:
                pr.append(i)
            pr.append(1)
            V2.append(pr)
            self.minCal1(P3.pos,P3.ang)  # Calulates the solution to the rotation error between the TF World->Gate_prism and TF of Leica->Gate_Prism
                
            # if(count<=6):
            #     count = count +1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.prismG3.setEnabled(False)

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
            # global count
            # if(count==4):
            P4.ang = resp.ang # Storage in anothe variable incase of flexibility with data if needed 
            P4.pos = resp.pos
            for i in P4.pos:
                pr.append(i)
            pr.append(1)
            V4.append(pr)
            self.minCal2(P4.pos,P4.ang)
            print("Prism no.:",count)
            print("***********Leica->Left Robot Prism***********")
            print(P4.pos)
            print(P4.ang)
            print("***********************************************")
            
            print('\n')
            # if(count<=6):
            #     count = count +1

            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.prismR1.setEnabled(False)

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
            # global count
            # if(count==5):
            P5.ang = resp.ang
            P5.pos = resp.pos
            for i in P5.pos:
                pr.append(i)
            pr.append(1)
            V4.append(pr)
            self.minCal2(P5.pos,P5.ang)
            print("Prism no.:",count)
            print("***********Leica -> Top Right Prism***********")
            print(P5.pos)
            print(P5.ang)
            print("***********************************************")
            
            print('\n')
            # if(count<=6):
            #     count = count +1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.prismR2.setEnabled(False)

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
            # global count
            # if(count==6):
            P6.ang = resp.ang
            P6.pos = resp.pos

            print("Prism no.:",count)
            print("***********Leica->Right Bottom Prism***********")
            print(P6.pos)
            print(P6.ang)

            for i in P6.pos:
                pr.append(i)
            pr.append(1)

            V4.append(pr)
            self.minCal2(P6.pos,P6.ang) # Calulates the solution to the rotation error between the TF Base_link->Robot_prism and TF of Leica->Robot_Prism
                
            # if(count<=6):
            #     count = count +1
            # if(count>6):
            #     print("Prism Count limit reached")
            #     count = 0        
            # self.World_Robot_Origin()         
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.prismR3.setEnabled(False)

    def btnQuit_onclick(self):
        self.pub_thread.join()
        self.parent().close()

    def pubTF(self):   #Robot_origin->World Publisher
        global min1,min2,V2,V4 

        while len(V2)<3 and len(V4)<3 and not rospy.is_shutdown():
            time.sleep(1)

        print("printing min1")
        print(min1)

        print("Prininting min2")
        print(min2)

        transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(tf.transformations.translation_from_matrix(min1)), tf.transformations.quaternion_matrix(normalize_quaternion(tf.transformations.quaternion_from_matrix(min1))))
        inversed_transform = tf.transformations.inverse_matrix(transform)

        print("Inverted Transform is ")
        print(inversed_transform)
        trans_inv_1 = tf.transformations.translation_from_matrix(inversed_transform)
        print("inversed Translation matrix")
        print(trans_inv_1)
        rot_inv_1 = tf.transformations.quaternion_from_matrix(inversed_transform)
        tf.transformations

        print("Inverted rotation")
        print(rot_inv_1)

        tnv1 = tf.transformations.translation_from_matrix(min2)
        rnv1 = tf.transformations.quaternion_from_matrix(min2)
        trans, rot = mtf(tnv1, rnv1, trans_inv_1, rot_inv_1)

        print("Multiplied Translation")
        print(trans)
        print("Multiplied Rotation" )
        print(rot)

        tv_mat = tf.transformations.translation_matrix(trans)
        rv_mat   = tf.transformations.quaternion_matrix(rot)

        print("Transform Robot with respect to World")
        mat = np.dot(tv_mat, rv_mat)
        print(mat)

        print("World with Respect to Robot")
        print(mat)

        while not rospy.is_shutdown():
            LeicaBroadcaster  = tf.TransformBroadcaster()
            current_time = rospy.Time.now()
            LeicaBroadcaster.sendTransform(
                tf.transformations.translation_from_matrix(mat), 
                tf.transformations.quaternion_from_matrix(mat),
                current_time, 
                "gate_leica", "body_aligned_imu_link") #Child Frame , Parent Frame 
            
            time.sleep(0.5)

        rospy.spin()


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










