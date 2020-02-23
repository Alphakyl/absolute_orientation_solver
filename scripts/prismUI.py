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
from marble_origin_detection_msgs.srv import *
# from marble_origin_detection_msgs.srv import SetTF, SetTFRequest, SetTFResponse

import threading

# GLOBAL VARIABLES
robot_ns = "A99"
child_frame_id = "gate_leica"
parent_frame_id = "body_aligned_imu_link"
baseB1name = "ugv"
gateG1name = "alpha"
# Urban: Alpha gate
# Vgp1 = [0.4425,1.3275 , 0.844]
# Vgp2 = [0.4535, -0.001, 2.741]
# Vgp3 = [0.462, -1.3115, 0.846]

# Urban: Beta gate
# Vgp1 = [0.4545, 1.324, 0.8445]
# Vgp2 = [0.4265, 0.002, 2.688]
# Vgp3 = [0.441, -1.3405, 0.8425]

# large test gate
# Vgp1 = [0.0,1.523 , 1.07]
# Vgp2 = [0.0, 0.0, 1.83]
# Vgp3 = [0.0, -1.394, 0.755]

# small test gate
# Vgp1 = [-0.045, 0.100025, 0] # left
# Vgp2 = [0.045, -0.100025, 0] # top
# Vgp3 = [-0.045, -0.100025, 0] # right

# UGVs w/ base plate v5
# Vrq1 = [-0.045, 0.100025, -0.0045]
# Vrq2 = [0.045, -0.100025, -0.0045]
# Vrq3 = [-0.045, -0.100025, -0.0045]

# UAVs on launch pad
# Vrq1 = [-0.2, 0.25, -0.25]
# Vrq2 = [0.2, -0.25, -0.25]
# Vrq3 = [-0.2, -0.25, -0.25]

# UAVs on launch pad corrected
# Vrq1 = [-0.25, -.1, -.205]
# Vrq2 = [0.25,0.1, -.205]
# Vrq3 = [0.25,-.1, -.205]

# Vgp = [Vgp1,Vgp2,Vgp3]
# Vrq = [Vrq1,Vrq2,Vrq3]
Vgp = [[None]*3 for i in range(3)]
Vrq = [[None]*3 for i in range(3)]
Vlp = [[None]*3 for i in range(3)]
Vlq = [[None]*3 for i in range(3)]

AVAILABLE_PRISMS = [
    {
        "name"     : "big_360",
        "num"      : 3,
        "constant" : 34.4,
        "z"        : 0.0
    },
    {
        "name"     : "mini_360",
        "num"      : 7,
        "constant" : 30.0, 
        "z"        : 0.02159,
    },
    {
        "name"     : "micro_360",
        "num"      : 7,
        "constant" : 30.0,
        "z"        : 0.01524,
    },
    {
        "name"     : "mini_lp",
        "num"      : 1,
        "constant" : 17.5,
        "z"        : 0.0,
    }
]

AVAILABLE_ROBOTS = ["H01","H02","H03","T01","T02","T03","L01","A01","A02","A03","A99"]

# List of base dictionaries added by Kyle
AVAILABLE_BASE = [
    {
        "name"      : "ugv",
        "Vrq1"      : [-0.045, 0.100025, -0.0045],
        "Vrq2"      : [0.045, -0.100025, -0.0045],
        "Vrq3"      : [-0.045, -0.100025, -0.0045],
    },
    {
        "name"      : "uav",
        "Vrq1"      : [-0.25, -.1, -.205],
        "Vrq2"      : [0.25,0.1, -.205],
        "Vrq3"      : [0.25,-.1, -.205],
    }
]

# List of gate dictionaries added by Kyle
AVAILABLE_GATES = [
    {
        "name"      : "alpha",
        "Vgp1"      : [0.4425,1.3275 , 0.844],
        "Vgp2"      : [0.4535, -0.001, 2.741],
        "Vgp3"      : [0.462, -1.3115, 0.846],
    },
    {
        "name"      : "beta",
        "Vgp1"      : [0.4545, 1.324, 0.8445],
        "Vgp2"      : [0.4265, 0.002, 2.688],
        "Vgp3"      : [0.441, -1.3405, 0.8425],
    },
    {
        "name"      : "small",
        "Vgp1"      : [-0.045, 0.100025, 0], # left
        "Vgp2"      : [0.045, -0.100025, 0], # top
        "Vgp3"      : [-0.045, -0.100025, 0], # right
    }
]

# Define Vgp and Vrq added by Kyle

for i in range(len(AVAILABLE_GATES)):
    if AVAILABLE_PRISMS[i]["name"]==gateG1name:
        Vgp = [AVAILABLE_GATES[i]["Vgp1"], AVAILABLE_GATES[i]["Vgp2"], AVAILABLE_GATES[i]["Vgp3"]]
        break
    else:
        rospy.logerror("Gate name does not exist")

for i in range(len(AVAILABLE_BASE)):
    if AVAILABLE_BASE[i]["name"]==baseB1name:
        Vrq = [AVAILABLE_BASE[i]["Vrq1"], AVAILABLE_GATES[i]["Vrq2"], AVAILABLE_GATES[i]["Vrq3"]]
        break
    else:
        rospy.logerror("Base name does not exist")

def cost_fun(x,v1,v2):
    v1 = map(list, zip(*v1))
    v2 = map(list, zip(*v2))
    # print v1
    # print v2
    t_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
    q_mat = tf.transformations.quaternion_matrix(normalize_quaternion(np.array([x[3], x[4], x[5],x[6]])))
    mat = np.dot(t_mat, q_mat)
    err_mat = v1-np.dot(mat,v2)
    # print err_mat
    err_norm = LA.norm(err_mat[:,0])+LA.norm(err_mat[:,1])+LA.norm(err_mat[:,2])   
    return err_norm

def cost_funz(x,v1,v2):
    v1 = map(list, zip(*v1))
    v2 = map(list, zip(*v2))
    # print v1
    # print v2
    t_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
    q_mat = tf.transformations.quaternion_matrix(normalize_quaternion(np.array([0, 0, x[3], 1-x[3]*x[3]])))
    mat = np.dot(t_mat, q_mat)
    err_mat = v1-np.dot(mat,v2)
    # print err_mat
    err_norm = LA.norm(err_mat[:,0])+LA.norm(err_mat[:,1])+LA.norm(err_mat[:,2])   
    return err_norm

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

class PrismMonitorWidget(QWidget): 
    commandUpdate = pyqtSignal(Transform)
    # Current_P = prism_tf()
    # counter = 0

    def __init__(self,parent = None):
        global robot_ns, baseB1name, gateG1name
        # self.counter = 0
        # self.Current_P = prism_tf()

        super(PrismMonitorWidget,self).__init__()
        layout = QVBoxLayout()

        # Begin GUI edits added by Kyle
        # initial group
        initialGroupLayout = QVBoxLayout()
        initialGroup = QGroupBox('Initialization')

        # initial base
        ibaseGroupLayout = QHBoxLayout()
        ibaseGroup = QGroupBox('Initial Base')
        self.baseB1toggle = QPushButton('Toggle')
        self.baseB1toggle.clicked.connect(self.baseB1toggle_onclick)
        boxLayout.addWidget(self.baseB1toggle)

        self.baselabel = QLabel(baseB1name)
        boxLayout.addWidget(self.baselabel)
        
        # initial gate
        igateGroupLayout = QHBoxLayout()
        igateGroup = QGroubBox()
        self.gateG1toggle = QPushButton('Toggle')
        self.gateG1toggle.clicked.connect(self.gateG1toggle_onclick)
        boxLayout.addWidget(self.gateG1toggle)

        self.gatelabel = Qlabel(gateG1name)
        boxLayout.addWidget(self.gatelabel)

        # gate group
        prismGroupLayout = QVBoxLayout()
        prismGroup = QGroupBox('Gate') 

        #Prism 1
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Left') 

        self.prismG1 = QPushButton('Calculate')
        self.prismG1.clicked.connect(self.prismG1_onclick) 
        boxLayout.addWidget(self.prismG1)

        self.prismG1toggle = QPushButton('Toggle')
        self.prismG1toggle.clicked.connect(self.prismG1toggle_onclick) 
        boxLayout.addWidget(self.prismG1toggle)

        self.prismG1name = "mini_lp"
        self.prismG1namelabel = QLabel(self.prismG1name) 
        boxLayout.addWidget(self.prismG1namelabel)

        groupBox.setLayout(boxLayout)
        prismGroupLayout.addWidget(groupBox)
        # layout.addWidget(groupBox)

        #Prism 2
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Top') 

        self.prismG2 = QPushButton('Calculate') 
        self.prismG2.clicked.connect(self.prismG2_onclick) 
        boxLayout.addWidget(self.prismG2)

        self.prismG2toggle = QPushButton('Toggle')
        self.prismG2toggle.clicked.connect(self.prismG2toggle_onclick) 
        boxLayout.addWidget(self.prismG2toggle)

        self.prismG2name = "mini_lp"
        self.prismG2namelabel = QLabel(self.prismG2name) 
        boxLayout.addWidget(self.prismG2namelabel)  

        groupBox.setLayout(boxLayout)
        prismGroupLayout.addWidget(groupBox)
        # layout.addWidget(groupBox)
        
        #Prism 3
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Right') 

        self.prismG3 = QPushButton('Calculate') 
        self.prismG3.clicked.connect(self.prismG3_onclick) 
        boxLayout.addWidget(self.prismG3)

        self.prismG3toggle = QPushButton('Toggle')
        self.prismG3toggle.clicked.connect(self.prismG3toggle_onclick) 
        boxLayout.addWidget(self.prismG3toggle)

        self.prismG3name = "mini_lp"
        self.prismG3namelabel = QLabel(self.prismG3name) 
        boxLayout.addWidget(self.prismG3namelabel)  

        groupBox.setLayout(boxLayout)
        prismGroupLayout.addWidget(groupBox)
        # layout.addWidget(groupBox)

        # reset gate
        self.btnResetGate = QPushButton('Reset Gate')
        self.btnResetGate.clicked.connect(self.btnResetGate_onclick)
        prismGroupLayout.addWidget(self.btnResetGate)

        # gate group
        prismGroup.setLayout(prismGroupLayout)
        layout.addWidget(prismGroup)

        # robot group
        prismGroupLayout = QVBoxLayout()
        prismGroup = QGroupBox('Robot')

        #Prism 4
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Left') 

        self.prismR1 = QPushButton('Calculate')
        self.prismR1.clicked.connect(self.prismR1_onclick) 
        boxLayout.addWidget(self.prismR1)

        self.prismR1toggle = QPushButton('Toggle')
        self.prismR1toggle.clicked.connect(self.prismR1toggle_onclick) 
        boxLayout.addWidget(self.prismR1toggle)

        self.prismR1name = "mini_360"
        self.prismR1namelabel = QLabel(self.prismR1name) 
        boxLayout.addWidget(self.prismR1namelabel)

        groupBox.setLayout(boxLayout)
        prismGroupLayout.addWidget(groupBox)
        # layout.addWidget(groupBox)

        #Prism 5
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Top') 

        self.prismR2 = QPushButton('Calculate') 
        self.prismR2.clicked.connect(self.prismR2_onclick) 
        boxLayout.addWidget(self.prismR2)

        self.prismR2toggle = QPushButton('Toggle')
        self.prismR2toggle.clicked.connect(self.prismR2toggle_onclick) 
        boxLayout.addWidget(self.prismR2toggle)

        self.prismR2name = "mini_360"
        self.prismR2namelabel = QLabel(self.prismR2name) 
        boxLayout.addWidget(self.prismR2namelabel)  

        groupBox.setLayout(boxLayout)
        prismGroupLayout.addWidget(groupBox)
        # layout.addWidget(groupBox)
        
        #Prism 6
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox("Right") 

        self.prismR3 = QPushButton('Calculate') 
        self.prismR3.clicked.connect(self.prismR3_onclick) 
        boxLayout.addWidget(self.prismR3)

        self.prismR3toggle = QPushButton('Toggle')
        self.prismR3toggle.clicked.connect(self.prismR3toggle_onclick) 
        boxLayout.addWidget(self.prismR3toggle)

        self.prismR3name = "mini_360"
        self.prismR3namelabel = QLabel(self.prismR3name) 
        boxLayout.addWidget(self.prismR3namelabel)  

        groupBox.setLayout(boxLayout)
        prismGroupLayout.addWidget(groupBox)
        # layout.addWidget(groupBox)

        self.btnResetRobot = QPushButton('Reset Robot')
        self.btnResetRobot.clicked.connect(self.btnResetRobot_onclick)
        prismGroupLayout.addWidget(self.btnResetRobot)

        # robot group
        prismGroup.setLayout(prismGroupLayout)
        layout.addWidget(prismGroup)

        # send tf and toggle robot layout
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox('Target Robot') 

        self.robottoggle = QPushButton('Toggle')
        self.robottoggle.clicked.connect(self.robottoggle_onclick) 
        boxLayout.addWidget(self.robottoggle)

        self.robotnslabel = QLabel(robot_ns) 
        boxLayout.addWidget(self.robotnslabel)

        self.sendtfbtn = QPushButton('Send TF')
        self.sendtfbtn.clicked.connect(self.sendTF) 
        boxLayout.addWidget(self.sendtfbtn)

        groupBox.setLayout(boxLayout)
        layout.addWidget(groupBox)
       
        #Exit Button layout
        self.btnQuit = QPushButton('Exit')
        self.btnQuit.clicked.connect(self.btnQuit_onclick)
        layout.addWidget(self.btnQuit)


        self.setLayout(layout)

        self.Tgl_found = False
        self.Trl_found = False
        self.Trg_found = False
        self.need_to_reset_gate = False
        self.need_to_reset_robot = False
        
        # self.LeicaStopTracking()


        # launch publisher thread
        self.pub_thread = threading.Thread(target=self.calcTF, args=())
        self.pub_thread.start()

    def solveForT(self,v1,v2):
        # v1 = map(list, zip(*v1))
        # v1.append([1,1,1])
        # v2 = map(list, zip(*v2))
        # v2.append([1,1,1])
        v1 = [pt+[1] for pt in v1]
        v2 = [pt+[1] for pt in v2]
        # xo = np.array([0,0,0,0,0,0,1])
        xyzy = np.array([0, 0, 0, 0])
        solution = minimize(cost_funz,xyzy,method='L-BFGS-B',args=(v1,v2))
        solution = minimize(cost_fun,[solution.x[0],solution.x[1],solution.x[2],0,0,solution.x[3],1-solution.x[3]*solution.x[3]],method='L-BFGS-B',args=(v1,v2))
        tran_mat = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
        quat = np.array([solution.x[3],solution.x[4],solution.x[5],solution.x[6]])
        quat = normalize_quaternion(quat) #Vital for Correct Solution
        quat_mat = tf.transformations.quaternion_matrix(np.array(quat))
        T12 = tf.transformations.concatenate_matrices(tran_mat,quat_mat)

        # print v1
        # print v2
        # print T12

        return T12

    def LeicaSetPrismType(self,prism_name):
        # try:
        #     rospy.wait_for_service('leica_node/set_prism_type',timeout=5)
        # except rospy.ServiceException as e:
        #     rospy.logwarn("Service call failed: %s",e)
        #     return
        set_prism_type_svc = rospy.ServiceProxy('leica_node/set_prism_type', SetPrismType)
        try:
            rospy.loginfo("Setting to prism: %s",prism_name)
            set_prism_type_req = SetPrismTypeRequest()
            set_prism_type_req.name = prism_name
            set_prism_type_resp = set_prism_type_svc(set_prism_type_req)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s",e)
            return 
        rospy.loginfo("Prism set to %s",prism_name)
        return set_prism_type_resp

    def LeicaGetPrismType(self):
        # try rospy.wait_for_service('leica_node/get_prism_type',timeout=5):
        #     continue
        # except rospy.ServiceException as e:
        #     rospy.logwarn("Service call failed: %s",e)
        get_prism_type_svc = rospy.ServiceProxy('leica_node/get_prism_type', GetPrismType)
        try:
            rospy.loginfo("Getting current prism name")
            get_prism_type_resp = get_prism_type_svc()
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s",e)
            return 
        current_prism_name = get_prism_type_resp.name
        rospy.loginfo("Current prism: %s",current_prism_name)
        return current_prism_name

    def LeicaStartTracking(self):
        # try rospy.wait_for_service('leica_node/start_tracking',timeout=5):
        #     continue
        # except rospy.ServiceException as e:
        #     rospy.logwarn("Service call failed: %s",e)
        start_tracking_svc = rospy.ServiceProxy('leica_node/start_tracking', StartTracking)
        try:
            rospy.loginfo("Starting tracking")
            start_tracking_resp = start_tracking_svc()
            # rospy.loginfo("StartTrackingResponse: %s",start_tracking_resp.__str__())
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s",e)
            return
        rospy.loginfo("Tracking started")
        return start_tracking_resp

    def LeicaStopTracking(self):
        # try rospy.wait_for_service('leica_node/stop_tracking',timeout=5):
        #     continue
        # except rospy.ServiceException as e:
        #     rospy.logwarn("Service call failed: %s",e)
        stop_tracking_svc = rospy.ServiceProxy('leica_node/stop_tracking', SetBool)
        try:
            rospy.loginfo("Stopping tracking")
            stop_tracking_req = SetBoolRequest()
            stop_tracking_req.data = False
            stop_tracking_resp = stop_tracking_svc(stop_tracking_req)
            # rospy.loginfo("StopTrackingResponse: %s",stop_tracking_resp.__str__())
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s",e)
            return 
        rospy.loginfo("Tracking stopped")
        return stop_tracking_resp

    def LeicaGetPos(self): 
        pos = [0, 0, 0]
        try:
            msg = rospy.wait_for_message("leica_node/position", PointStamped, 5.0)
            pos = [msg.point.x, msg.point.y, msg.point.z]
        except rospy.exceptions.ROSException as e:
            rospy.logwarn("Service call failed: %s",e)
        return pos

    def getTFOnClick(self,prism_btn,prism_name):
        pos = [0,0,0]
           
        prism_btn.setEnabled(False)
        # global Vlp,Vgp
        # set prism type in Leica to that which is currently displayed in the GUI
        self.LeicaSetPrismType(prism_name)

        # # start Leica tracking
        # self.LeicaStartTracking()
        # # get tf
        # count = 0
        # pos = self.LeicaGetPos()
        # while any([i==0 for i in pos]):
        #     count+=1
        #     if count<5:
        #         pos = self.LeicaGetPos()
        #     else:
        #         rospy.logwarn("Cannot get pos from Leica. Aborting.")
        #         self.LeicaStopTracking()
        #         return
        # rospy.loginfo("Got pos: %s",pos)
        # # stop Leica tracking
        # self.LeicaStopTracking()

        got_pos = False
        no_fails = 0
        max_no_fails = 5
        while not got_pos:
            self.LeicaStartTracking()
            pos = self.LeicaGetPos()
            got_pos = not all([i==0 for i in pos])
            if not got_pos:
                no_fails += 1
                if no_fails<max_no_fails:
                    rospy.logwarn("Cannot get pos from Leica. Trying again (%d/%d attempts left).",max_no_fails-no_fails,max_no_fails)
                    rospy.logwarn("Possible causes: \n- target prism appears too close to another prism")
                else:
                    rospy.logwarn("Cannot get pos from Leica. Aborting.")
                    got_pos = True
                    prism_btn.setEnabled(True)
            self.LeicaStopTracking()
        return pos

    # Added by Kyle, gate type toggle
    def toggleGateType(self, current_gate_name):
        current_idx = next((i for i in range(len(AVAILABLE_GATES)) if AVAILABLE_GATES[i]["name"]==current_gate_name),None)
        if current_idx is None:
            rospy.logwarn("The current gate name is invalid")
            return
        next_idx = current_idx+1
        if next_idx >= len(AVAILABLE_GATES):
            next_idx = 0
        new_gate_name = AVAILABLE_GATES[next_idx]["name"]
        return new_gate_name

    # Added by Kyle, base type toggle 
    def toggleRobotBase(self,current_robot_base_name):
        current_idx = next((i for i in range(len(AVAILABLE_BASE)) if AVAILABLE_BASE[i]["name"]==current_base_name),None)
        if current_idx is None:
            rospy.logwarn("The current base name is invalid")
            return
        next_idx = current_idx+1
        if next_idx >= len(AVAILABLE_BASE):
            next_idx = 0
        new_base_name = AVAILABLE_BASE[next_idx]["name"]
        return new_base_name
    
    
    def togglePrismType(self,current_prism_name):
        # current_prism_name = self.LeicaGetPrismType()
        current_idx = next((i for i in range(len(AVAILABLE_PRISMS)) if AVAILABLE_PRISMS[i]["name"]==current_prism_name),None)
        if current_idx is None:
            rospy.logwarn("The current prism name is invalid")
            return
        next_idx = current_idx+1
        if next_idx >= len(AVAILABLE_PRISMS):
            next_idx = 0
        new_prism_name = AVAILABLE_PRISMS[next_idx]["name"]
        return new_prism_name

    def toggleRobot(self,current_robot_name):
        current_idx = next((i for i in range(len(AVAILABLE_ROBOTS)) if AVAILABLE_ROBOTS[i]==current_robot_name),None)
        if current_idx is None:
            rospy.logwarn("The current robot name is invalid")
            return
        next_idx = current_idx+1
        if next_idx >= len(AVAILABLE_ROBOTS):
            next_idx = 0
        new_robot_name = AVAILABLE_ROBOTS[next_idx]
        return new_robot_name
    
    # on click gate added by Kyle
    def gateG1toggle_onclick(self):
        global gateG1name, Vgp
        gateG1name = self.toggleGateType(gateG1name)
        for i in range(len(AVAILABLE_GATES)):
            if AVAILABLE_PRISMS[i]["name"]==gateG1name:
                Vgp = [AVAILABLE_GATES[i]["Vgp1"], AVAILABLE_GATES[i]["Vgp2"], AVAILABLE_GATES[i]["Vgp3"]]
                break
            else:
                rospy.logerror("Gate name does not exist")
        self.gatelabel.setText(gateG1name)
    
    # on click base added by Kyle
    def baseB1toggle_onclick(self):
        global baseB1name, Vrq
        baseB1name = self.toggleRobotBase(baseB1name)
        for i in range(len(AVAILABLE_BASE)):
            if AVAILABLE_BASE[i]["name"]==baseB1name:
                Vrq = [AVAILABLE_BASE[i]["Vrq1"], AVAILABLE_GATES[i]["Vrq2"], AVAILABLE_GATES[i]["Vrq3"]]
                break
            else:
                rospy.logerror("Base name does not exist")
        self.baselabel.setText(baseB1name)
        

    def robottoggle_onclick(self):
        global robot_ns
        robot_ns = self.toggleRobot(robot_ns)
        self.robotnslabel.setText(robot_ns)

    def prismG1toggle_onclick(self):
        self.prismG1name = self.togglePrismType(self.prismG1name)
        self.prismG1namelabel.setText(self.prismG1name)

    def prismG1_onclick(self):
        global Vlp, Vgp
        rospy.loginfo("Calculating Gate Position 1")
        pos = self.getTFOnClick(self.prismG1,self.prismG1name)
        if not all([i==0 for i in pos]):
            Vlp[0] = pos

        delta_z = AVAILABLE_PRISMS[next(i for i in range(len(AVAILABLE_PRISMS)) if AVAILABLE_PRISMS[i]["name"]==self.prismG1name)]["z"]
        if isinstance(delta_z,list):
            rospy.logerror("Multiple available prisms of same name.")
            return pos
        Vgp[0][2] += delta_z

    def prismG2toggle_onclick(self):
        self.prismG2name = self.togglePrismType(self.prismG2name)
        self.prismG2namelabel.setText(self.prismG2name)

    def prismG2_onclick(self):
        global Vlp, Vgp
        rospy.loginfo("Calculating Gate Position 2")
        pos = self.getTFOnClick(self.prismG2,self.prismG2name)
        if not all([i==0 for i in pos]):
            Vlp[1] = pos

        delta_z = AVAILABLE_PRISMS[next(i for i in range(len(AVAILABLE_PRISMS)) if AVAILABLE_PRISMS[i]["name"]==self.prismG2name)]["z"]
        if isinstance(delta_z,list):
            rospy.logerror("Multiple available prisms of same name.")
            return pos
        Vgp[1][2] += delta_z

    def prismG3toggle_onclick(self):
        self.prismG3name = self.togglePrismType(self.prismG3name)
        self.prismG3namelabel.setText(self.prismG3name)

    def prismG3_onclick(self):
        global Vlp, Vgp
        rospy.loginfo("Calculating Gate Position 3")
        pos = self.getTFOnClick(self.prismG3,self.prismG3name)
        if not all([i==0 for i in pos]):
            Vlp[2] = pos

        delta_z = AVAILABLE_PRISMS[next(i for i in range(len(AVAILABLE_PRISMS)) if AVAILABLE_PRISMS[i]["name"]==self.prismG3name)]["z"]
        if isinstance(delta_z,list):
            rospy.logerror("Multiple available prisms of same name.")
            return pos
        Vgp[2][2] += delta_z

    def prismR1toggle_onclick(self):
        self.prismR1name = self.togglePrismType(self.prismR1name)
        self.prismR1namelabel.setText(self.prismR1name)

    def prismR1_onclick(self):
        global Vlq, Vrq
        rospy.loginfo("Calculating Robot Position 1")
        pos = self.getTFOnClick(self.prismR1,self.prismR1name)
        if not all([i==0 for i in pos]):
            Vlq[0] = pos

        delta_z = AVAILABLE_PRISMS[next(i for i in range(len(AVAILABLE_PRISMS)) if AVAILABLE_PRISMS[i]["name"]==self.prismR1name)]["z"]
        if isinstance(delta_z,list):
            rospy.logerror("Multiple available prisms of same name.")
            return pos
        Vrq[0][2] += delta_z

    def prismR2toggle_onclick(self):
        self.prismR2name = self.togglePrismType(self.prismR2name)
        self.prismR2namelabel.setText(self.prismR2name)

    def prismR2_onclick(self):
        global Vlq, Vrq
        rospy.loginfo("Calculating Robot Position 2")
        pos = self.getTFOnClick(self.prismR2,self.prismR2name)
        if not all([i==0 for i in pos]):
            Vlq[1] = pos

        delta_z = AVAILABLE_PRISMS[next(i for i in range(len(AVAILABLE_PRISMS)) if AVAILABLE_PRISMS[i]["name"]==self.prismR2name)]["z"]
        if isinstance(delta_z,list):
            rospy.logerror("Multiple available prisms of same name.")
            return pos
        Vrq[1][2] += delta_z

    def prismR3toggle_onclick(self):
        self.prismR3name = self.togglePrismType(self.prismR3name)
        self.prismR3namelabel.setText(self.prismR3name)

    def prismR3_onclick(self):
        global Vlq, Vrq
        rospy.loginfo("Calculating Robot Position 3")
        pos = self.getTFOnClick(self.prismR3,self.prismR3name)
        if not all([i==0 for i in pos]):
            Vlq[2] = pos

        delta_z = AVAILABLE_PRISMS[next(i for i in range(len(AVAILABLE_PRISMS)) if AVAILABLE_PRISMS[i]["name"]==self.prismR3name)]["z"]
        if isinstance(delta_z,list):
            rospy.logerror("Multiple available prisms of same name.")
            return pos
        Vrq[2][2] += delta_z

    def btnResetGate_onclick(self):
        self.need_to_reset_gate = True

    def btnResetRobot_onclick(self):
        self.need_to_reset_robot = True

    def resetGate(self):
        global Vlp
        rospy.loginfo("Resetting Gate")

        self.LeicaStopTracking()

        Vlp = [[None]*3 for i in range(3)]
        self.Trg = None

        self.Tgl_found = False
        self.Trg_found = False

        self.prismG1.setEnabled(True)
        self.prismG2.setEnabled(True)
        self.prismG3.setEnabled(True)

        self.need_to_reset_gate = False

    
    def resetRobot(self):
        global Vlq
        rospy.loginfo("Resetting Robot")

        self.LeicaStopTracking()

        Vlq = [[None]*3 for i in range(3)]
        self.Trg = None

        self.Trl_found = False
        self.Trg_found = False

        self.prismR1.setEnabled(True)
        self.prismR2.setEnabled(True)
        self.prismR3.setEnabled(True)

        self.need_to_reset_robot = False

    def btnQuit_onclick(self):
        self.pub_thread.join()
        self.parent().close()

    def calcTF(self):   #Robot_origin->World Publisher
        global Vgp,Vlp,Vrq,Vlq,Tgr

        if len(Vgp)<3 or len(Vrq)<3:
            rospy.logerror("Invalid ground truth parameters")
            return

        while not rospy.is_shutdown():
            if self.need_to_reset_gate:
                self.resetGate()
            if self.need_to_reset_robot:
                self.resetRobot()
            if not any(None in pt for pt in Vlp) and not self.Tgl_found:
                Tgl = self.solveForT(Vgp,Vlp)
                rospy.loginfo("Gate->Leica:\n%s, %s",\
                    tf.transformations.translation_from_matrix(Tgl).__str__(),\
                    [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(Tgl, 'sxyz')].__str__())
                self.Tgl_found = True
            if not any(None in pt for pt in Vlq) and not self.Trl_found:
                Trl = self.solveForT(Vrq,Vlq)
                rospy.loginfo("Robot->Leica:\n%s, %s",\
                    tf.transformations.translation_from_matrix(Trl).__str__(),\
                    [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(Trl, 'sxyz')].__str__())
                self.Trl_found = True
            if self.Tgl_found and self.Trl_found:
                Tgr = tf.transformations.concatenate_matrices(Tgl,tf.transformations.inverse_matrix(Trl))
                Trg = tf.transformations.inverse_matrix(Tgr)
                self.Trg = Trg
                if not self.Trg_found:
                    rospy.loginfo("Robot->Gate:\n%s, %s",\
                        tf.transformations.translation_from_matrix(Trg).__str__(),\
                        [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(Trg, 'sxyz')].__str__())
                self.Trg_found = True
            time.sleep(1)

    def sendTF(self): 
        global child_frame_id, parent_frame_id, robot_ns

        if not self.Trg_found:
            rospy.logwarn("Can't send TF. Tf not found yet.")
            return
        if not self.Trg.shape==(4,4):
            rospy.logwarn("Can't send TF. Tf is wrong size.")
            return

        # wait_for_svc_timeout = 5.0
        svc_name = "/"+robot_ns+"/"+"set_world_tf"
        rospy.loginfo("Sending tf to %s",svc_name)
       
        # rospy.loginfo("Waiting for SetTF service.")
        # if rospy.wait_for_service(svc_name,timeout=wait_for_svc_timeout):
        set_tf_svc = rospy.ServiceProxy(svc_name, SetTF)
        set_tf_resp = SetTFResponse()
        try:
            # rospy.loginfo("Setting Robot->Gate:\n%s",T.__str__())

            tf_msg = TransformStamped()
            tf_msg.header.stamp = rospy.Time.now()
            tf_msg.header.frame_id = parent_frame_id
            tf_msg.child_frame_id = child_frame_id
            tf_msg.transform.translation.x = tf.transformations.translation_from_matrix(self.Trg)[0]
            tf_msg.transform.translation.y = tf.transformations.translation_from_matrix(self.Trg)[1]
            tf_msg.transform.translation.z = tf.transformations.translation_from_matrix(self.Trg)[2]
            tf_msg.transform.rotation.x = tf.transformations.quaternion_from_matrix(self.Trg)[0]
            tf_msg.transform.rotation.y = tf.transformations.quaternion_from_matrix(self.Trg)[1]
            tf_msg.transform.rotation.z = tf.transformations.quaternion_from_matrix(self.Trg)[2]
            tf_msg.transform.rotation.w = tf.transformations.quaternion_from_matrix(self.Trg)[3]

            set_tf_req = SetTFRequest()
            set_tf_req.transform = tf_msg
            set_tf_resp = set_tf_svc(set_tf_req)

            if set_tf_resp:
                rospy.loginfo("TF sent.")
            else:
                rospy.logwarn("Service call failed.")

        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s",e)
            return

        # else:
        #     rospy.logwarn("Failed to send TF after "+wait_for_svc_timeout.__str__()+" s.")
        #     return
        return

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










