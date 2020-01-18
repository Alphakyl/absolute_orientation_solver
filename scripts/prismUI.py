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

# GLOBAL VARIABLES
child_frame_id = "gate_leica"
parent_frame_id = "body_aligned_imu_link"

# Vgp1 = [0.0,1.523 , 1.07]
# Vgp2 = [0.0, 0.0, 1.83]
# Vgp3 = [0.0, -1.394, 0.755]
Vgp1 = [-0.045, 0.100025, 0.0174602]
Vgp2 = [0.045, -0.100025, 0.0174602]
Vgp3 = [-0.045, -0.100025, 0.0174602]
Vgp = [Vgp1,Vgp2,Vgp3]

Vrq1 = [-0.045, 0.100025, 0.0174602]
Vrq2 = [0.045, -0.100025, 0.0174602]
Vrq3 = [-0.045, -0.100025, 0.0174602]
Vrq = [Vrq1,Vrq2,Vrq3]

Vlp = []
Vlq = []

def cost_fun(x,v1,v2):
    t_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
    q_mat = tf.transformations.quaternion_matrix(normalize_quaternion(np.array([x[3], x[4], x[5],x[6]])))
    mat = np.dot(t_mat, q_mat)
    # print v1
    # print v2
    # print mat
    err_mat = v1-np.dot(mat,v2)
    err_norm = LA.norm(err_mat[0,:])+LA.norm(err_mat[1,:])+LA.norm(err_mat[2,:])   
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
        # self.counter = 0
        # self.Current_P = prism_tf()

        super(PrismMonitorWidget,self).__init__()
        layout = QVBoxLayout()

        #Prism 1
        prismLayout = QHBoxLayout()
        self.controlGroup = QGroupBox('Prism Left Gate') 

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
        prismLayout = QHBoxLayout()
        self.controlGroup = QGroupBox('Prism Top Gate') 

        self.prismG2 = QPushButton('Calculate') 
        self.prismG2.clicked.connect(self.prismG2_onclick) 
        prismLayout.addWidget(self.prismG2)

        self.prismG2toggle = QPushButton('Toggle')
        self.prismG2toggle.clicked.connect(self.prismG2toggle_onclick) 
        prismLayout.addWidget(self.prismG2toggle)

        self.prismG2name = "mini_lp"
        self.prismG2namelabel = QLabel(self.prismG2name) 
        prismLayout.addWidget(self.prismG2namelabel)  

        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
        
        #Prism 3
        prismLayout = QHBoxLayout()
        self.controlGroup = QGroupBox('Prism Right Gate') 

        self.prismG3 = QPushButton('Calculate') 
        self.prismG3.clicked.connect(self.prismG3_onclick) 
        prismLayout.addWidget(self.prismG3)

        self.prismG3toggle = QPushButton('Toggle')
        self.prismG3toggle.clicked.connect(self.prismG3toggle_onclick) 
        prismLayout.addWidget(self.prismG3toggle)

        self.prismG3name = "mini_lp"
        self.prismG3namelabel = QLabel(self.prismG3name) 
        prismLayout.addWidget(self.prismG3namelabel)  

        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
        
        #Prism 4
        prismLayout = QHBoxLayout()
        self.controlGroup = QGroupBox('Prism Left Robot') 

        self.prismR1 = QPushButton('Calculate')
        self.prismR1.clicked.connect(self.prismR1_onclick) 
        prismLayout.addWidget(self.prismR1)

        self.prismR1toggle = QPushButton('Toggle')
        self.prismR1toggle.clicked.connect(self.prismR1toggle_onclick) 
        prismLayout.addWidget(self.prismR1toggle)

        self.prismR1name = "mini_360"
        self.prismR1namelabel = QLabel(self.prismR1name) 
        prismLayout.addWidget(self.prismR1namelabel)

        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)

        #Prism 5
        prismLayout = QHBoxLayout()
        self.controlGroup = QGroupBox('Prism Top Robot') 

        self.prismR2 = QPushButton('Calculate') 
        self.prismR2.clicked.connect(self.prismR2_onclick) 
        prismLayout.addWidget(self.prismR2)

        self.prismR2toggle = QPushButton('Toggle')
        self.prismR2toggle.clicked.connect(self.prismR2toggle_onclick) 
        prismLayout.addWidget(self.prismR2toggle)

        self.prismR2name = "mini_360"
        self.prismR2namelabel = QLabel(self.prismR2name) 
        prismLayout.addWidget(self.prismR2namelabel)  

        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
        
        #Prism 6
        prismLayout = QHBoxLayout()
        self.controlGroup = QGroupBox('Prism Right Robot') 

        self.prismR3 = QPushButton('Calculate') 
        self.prismR3.clicked.connect(self.prismR3_onclick) 
        prismLayout.addWidget(self.prismR3)

        self.prismR3toggle = QPushButton('Toggle')
        self.prismR3toggle.clicked.connect(self.prismR3toggle_onclick) 
        prismLayout.addWidget(self.prismR3toggle)

        self.prismR3name = "mini_360"
        self.prismR3namelabel = QLabel(self.prismR3name) 
        prismLayout.addWidget(self.prismR3namelabel)  

        self.controlGroup.setLayout(prismLayout)
        layout.addWidget(self.controlGroup)
       
        #Exit Button layout
        self.btnQuit = QPushButton('Exit')
        self.btnQuit.clicked.connect(self.btnQuit_onclick)
        layout.addWidget(self.btnQuit)

        self.setLayout(layout)
        
        self.LeicaStopTracking()

        # launch publisher thread
        self.pub_thread = threading.Thread(target=self.calcTF, args=())
        self.pub_thread.start()
    
    def solveForT(self,v1,v2):
        v1 = map(list, zip(*v1))
        v1.append([1,1,1])
        v2 = map(list, zip(*v2))
        v2.append([1,1,1])
        xo = np.array([0,0,0,0,0,0,1])
        solution = minimize(cost_fun,xo,method='L-BFGS-B',args=(v1,v2))
        tran_mat = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
        quat = np.array([solution.x[3],solution.x[4],solution.x[5],solution.x[6]])
        quat = normalize_quaternion(quat) #Vital for Correct Solution
        quat_mat = tf.transformations.quaternion_matrix(np.array(quat))
        T12 = tf.transformations.concatenate_matrices(tran_mat,quat_mat)
        return T12

    def LeicaSetPrismType(self,prism_name):
        rospy.wait_for_service('leica_node/set_prism_type')
        set_prism_type_svc = rospy.ServiceProxy('leica_node/set_prism_type', SetPrismType)
        try:
            rospy.loginfo("Setting to prism: %s",prism_name)
            set_prism_type_req = SetPrismTypeRequest()
            set_prism_type_req.name = prism_name
            set_prism_type_resp = set_prism_type_svc(set_prism_type_req)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
            return 
        rospy.loginfo("Prism set to %s",prism_name)
        return set_prism_type_resp

    def LeicaGetPrismType(self):
        rospy.wait_for_service('leica_node/get_prism_type')
        get_prism_type_svc = rospy.ServiceProxy('leica_node/get_prism_type', GetPrismType)
        try:
            rospy.loginfo("Getting current prism name")
            get_prism_type_resp = get_prism_type_svc()
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
            return 
        current_prism_name = get_prism_type_resp.name
        rospy.loginfo("Current prism: %s",current_prism_name)
        return current_prism_name

    def LeicaStartTracking(self):
        rospy.wait_for_service('leica_node/start_tracking')
        start_tracking_svc = rospy.ServiceProxy('leica_node/start_tracking', StartTracking)
        try:
            rospy.loginfo("Starting tracking")
            start_tracking_resp = start_tracking_svc()
            # rospy.loginfo("StartTrackingResponse: %s",start_tracking_resp.__str__())
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
            return
        rospy.loginfo("Tracking started")
        return start_tracking_resp

    def LeicaStopTracking(self):
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
        rospy.loginfo("Tracking stopped")
        return stop_tracking_resp

    def LeicaGetTF(self):
        tf_listener = tf.TransformListener()
        rospy.wait_for_service('prismTransform')
        try: 
            rospy.loginfo('Getting TF')
            prismTransform = rospy.ServiceProxy('prismTransform', gettf) # s = client object
            req = gettfRequest() 
            resp = prismTransform(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.loginfo("Got pos: %s",resp.pos.__str__())
        return resp.pos

    def getTFOnClick(self,prism_btn,prism_name):
        # global Vlp,Vgp
        # set prism type in Leica to that which is currently displayed in the GUI
        self.LeicaSetPrismType(prism_name)
        # start Leica tracking
        self.LeicaStartTracking()
        # get tf
        pos = self.LeicaGetTF()
        # stop Leica tracking
        self.LeicaStopTracking()
        # disable calculate button    
        prism_btn.setEnabled(False)

        return pos
     
    def togglePrismType(self,current_prism_name):
        # current_prism_name = self.LeicaGetPrismType()
        current_idx = next((i for i in range(len(self.availablePrisms)) if self.availablePrisms[i]["name"]==current_prism_name),None)
        if current_idx is None:
            rospy.logwarn("The current prism name is invalid")
            return
        next_idx = current_idx+1
        if next_idx >= len(self.availablePrisms):
            next_idx = 0
        new_prism_name = self.availablePrisms[next_idx]["name"]
        return new_prism_name

    def prismG1toggle_onclick(self):
        self.prismG1name = self.togglePrismType(self.prismG1name)
        self.prismG1namelabel.setText(self.prismG1name)

    def prismG1_onclick(self):
        global Vlp
        pos = self.getTFOnClick(self.prismG1,self.prismG1name)
        Vlp.append(list(pos))

    def prismG2toggle_onclick(self):
        self.prismG2name = self.togglePrismType(self.prismG2name)
        self.prismG2namelabel.setText(self.prismG2name)

    def prismG2_onclick(self):
        global Vlp
        pos = self.getTFOnClick(self.prismG2,self.prismG2name)
        Vlp.append(list(pos))

    def prismG3toggle_onclick(self):
        self.prismG3name = self.togglePrismType(self.prismG3name)
        self.prismG3namelabel.setText(self.prismG3name)

    def prismG3_onclick(self):
        global Vlp
        pos = self.getTFOnClick(self.prismG3,self.prismG3name)
        Vlp.append(list(pos))

    def prismR1toggle_onclick(self):
        self.prismR1name = self.togglePrismType(self.prismR1name)
        self.prismR1namelabel.setText(self.prismR1name)

    def prismR1_onclick(self):
        global Vlq
        pos = self.getTFOnClick(self.prismR1,self.prismR1name)
        Vlq.append(list(pos))

    def prismR2toggle_onclick(self):
        self.prismR2name = self.togglePrismType(self.prismR2name)
        self.prismR2namelabel.setText(self.prismR2name)

    def prismR2_onclick(self):
        global Vlq
        pos = self.getTFOnClick(self.prismR2,self.prismR2name)
        Vlq.append(list(pos))

    def prismR3toggle_onclick(self):
        self.prismR3name = self.togglePrismType(self.prismR3name)
        self.prismR3namelabel.setText(self.prismR3name)

    def prismR3_onclick(self):
        global Vlq
        pos = self.getTFOnClick(self.prismR3,self.prismR3name)
        Vlq.append(list(pos))

    def btnQuit_onclick(self):
        self.pub_thread.join()
        self.parent().close()

    def calcTF(self):   #Robot_origin->World Publisher
        global Vgp,Vlp,Vrq,Vlq,Tgr

        if len(Vgp)<3 or len(Vrq)<3:
            rospy.logerror("Invalid ground truth parameters")
            return

        Tgl_found = False
        Trl_found = False
        while not rospy.is_shutdown() and (not Tgl_found or not Trl_found):
            if len(Vlp)>=3 and not Tgl_found:
                # print Vgp
                # print Vlp
                Tgl = self.solveForT(Vgp,Vlp)
                rospy.loginfo("Gate->Leica:\n%s",Tgl.__str__())
                Tgl_found = True
            if len(Vlq)>=3 and not Trl_found:
                # print Vrq
                # print Vlq
                Trl = self.solveForT(Vrq,Vlq)
                rospy.loginfo("Robot->Leica:\n%s",Trl.__str__())
                Trl_found = True
            time.sleep(1)
            
        Tgr = tf.transformations.concatenate_matrices(Tgl,tf.transformations.inverse_matrix(Trl))
        # print Tgr
        self.pubTF()

        return

    def pubTF(self): 
        global child_frame_id, parent_frame_id, Tgr

        rospy.loginfo("Publishing Gate->Robot:\n%s",Tgr.__str__())

        Trg = tf.transformations.inverse_matrix(Tgr)

        LeicaBroadcaster  = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            LeicaBroadcaster.sendTransform(
                tf.transformations.translation_from_matrix(Trg), 
                tf.transformations.quaternion_from_matrix(Trg),
                rospy.Time.now(), 
                child_frame_id,parent_frame_id) #Child Frame , Parent Frame 
            time.sleep(0.5)

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










