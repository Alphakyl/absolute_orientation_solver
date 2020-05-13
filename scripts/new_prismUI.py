#!/usr/bin/python2
import sys
import rospy 
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from functools import partial
import tf_calc
import leica_service as LS
import tf

#import math
#import numpy as np 
#import tf2_ros
#from numpy import linalg as LA
#from numpy.linalg import inv
#from scipy.optimize import minimize
#from scipy.spatial.transform import Rotation
#import itertools as it
#import time
# from QLabeledValue import *
#from leica_ros.msg import *
#from std_msgs.msg import *
#from std_srvs.srv import *
#from geometry_msgs.msg import *

#import message_filters

#from leica_ros.srv import *
#from marble_origin_detection_msgs.srv import *
# from marble_origin_detection_msgs.srv import SetTF, SetTFRequest, SetTFResponse

#import threading

# TODO: Wire SEND TF BLOCK

# GLOBAL VARIABLES
robot_ns = "H01"
child_frame_id = "gate_leica"
parent_frame_id = "body_aligned_imu_link"
current_gate = "Small"
current_base = "UGV"
project_transform_from_3d_to_2d = False

# Dictionaries the hold informations on various prisms and bases
AVAILABLE_PRISMS = {
    "big_360": {
        "num"      : 3,
        "constant" : 34.4,
        "z"        : 0.0
        },
    "mini_360": {
        "num"      : 7,
        "constant" : 30.0, 
        "z"        : 0.02159
        },
    "micro_360": {
        "num"     : 7,
        "constant" : 30.0,
        "z"        : 0.01524
        },
    "mini_lp":  {
        "num"     : 1,
        "constant" : 17.5,
        "z"        : 0.0
        }
}

AVAILABLE_ROBOTS = ["H01","H02","H03","T01","T02","T03","L01","A01","A02","A03","A99"]

# List of base dictionaries added by Kyle
AVAILABLE_BASE = {
    "UGV": {
        "Vrq1"      : [-0.045, 0.100025, -0.0045],
        "Vrq2"      : [0.045, -0.100025, -0.0045],
        "Vrq3"      : [-0.045, -0.100025, -0.0045],
    },
    "UAV": {
        "Vrq1"      : [-0.25, -.1, -.205],
        "Vrq2"      : [0.25,0.1, -.205],
        "Vrq3"      : [0.25,-.1, -.205],
    }
}

# List of gate dictionaries added by Kyle
AVAILABLE_GATES = {
    "Alpha": {
        "Vgp1"      : [0.4425,1.3275 , 0.844],
        "Vgp2"      : [0.4535, -0.001, 2.741],
        "Vgp3"      : [0.462, -1.3115, 0.846],
    },
    "Beta": {
        "Vgp1"      : [0.4545, 1.324, 0.8445],
        "Vgp2"      : [0.4265, 0.002, 2.688],
        "Vgp3"      : [0.441, -1.3405, 0.8425],
    },
    "Small": {
        "Vgp1"      : [-0.045, 0.100025, 0], # left
        "Vgp2"      : [0.045, -0.100025, 0], # top
        "Vgp3"      : [-0.045, -0.100025, 0], # right
    }
}


# Empty global variables for storing prism points (Given values and calculated from the Leica)
V_gate_prism = [AVAILABLE_GATES[current_gate][key] for key in AVAILABLE_GATES[current_gate]]
V_robot_prism = [AVAILABLE_BASE[current_base][key] for key in AVAILABLE_BASE[current_base]]
V_leica_prism_gate = [[None]*3 for i in range(3)]
V_leica_prism_robot = [[None]*3 for i in range(3)]


class PrismMonitorWidget(QMainWindow):
    ##############################################################################
    # UI Stuff Here
    ###############################################################################
    def __init__(self,parent = None):
        # not sure what super does
        super(PrismMonitorWidget,self).__init__()
        # Create a vertical layout (separate stuff vertically)
        self.layout = QVBoxLayout()
        # Create the central widget
        self._centralWidget=QWidget(self)
        # Set the central widget to be the main widget
        self.setCentralWidget(self._centralWidget)
        # Define the layout of the central widget (vertical layou from above)
        self._centralWidget.setLayout(self.layout)

        # Create a holder dictionary for buttons and combo boxes
        self.buttons = {}
        self.prismOptions = {}

        # Define the Gate layout (First part of central widget)
        self._createGateCalculate()
        # Define the robot layout (Second part of the central widget)
        self._createRobotCalculate()
        # Define the target layout (Third part of central widget)
        self._createRobotTarget()
        # Create an exit button for convenience
        self._createExit()

        # Wire up buttons to do various functions
        self._connectSignals()

        # launch publisher thread
        # self.pub_thread = threading.Thread(target=self._calcTF, args=())
        # self.pub_thread.start()

    def _createButtonAndPrismComboBox(self, group_label, box_label):
        # Generic button and combo box layout for prism selection

        # Creates a horizontal layout
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox(box_label)

        # Creates a button and adds it to a dictionary of dictionaries for later use
        self.buttons[group_label][box_label] = QPushButton('Calculate')
        boxLayout.addWidget(self.buttons[group_label][box_label])

        # Creates a combo box and adds it to a dictionary of dictionaries for later use
        self.prismOptions[group_label][box_label] = QComboBox()
        l = []
        for key in AVAILABLE_PRISMS:
            l.append(key)
        self.prismOptions[group_label][box_label].addItems(l)

        # Adds layout to our widget
        boxLayout.addWidget(self.prismOptions[group_label][box_label])
        groupBox.setLayout(boxLayout)
        
        # Returns the layout for use outside this function
        return groupBox
    
    def _createGateCalculate(self):
        # Creating the gate functions in the GUI
        prismGroupLayout = QVBoxLayout()
        # Label overall box as Gate
        prismGroup = QGroupBox('Gate')

        # Add dropdown for Gate
        self.gateOption = QComboBox()
        l = []
        for key in AVAILABLE_GATES:
            l.append(key)
        self.gateOption.addItems(l)
        prismGroupLayout.addWidget(self.gateOption)
        self.gateOption.currentIndexChanged.connect(self._current_gate)    

        # Add buttons and dropdowns for each prism
        group_label = 'Gate'
        # Creates the outer key value as an empty dictionary
        self.buttons[group_label] = {}
        self.prismOptions[group_label] = {}
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox(group_label,'Left'))
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox(group_label,'Top'))
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox(group_label,'Right'))

        # Create a button to call Horn's Method
        self.btnSolveGate = QPushButton('Calculate G->L TF')
        self.btnSolveGate.clicked.connect(partial(self.Solve_onclick, 'Gate'))
        prismGroupLayout.addWidget(self.btnSolveGate)

        # reset gate
        self.btnResetGate = QPushButton('Reset G->L TF')
        self.btnResetGate.clicked.connect(partial(self.Reset_onclick, 'Gate'))
        prismGroupLayout.addWidget(self.btnResetGate)

        # Layout the gate group in the GUI
        prismGroup.setLayout(prismGroupLayout)
        self.layout.addWidget(prismGroup)

    def _createRobotCalculate(self):
        # Creating the robot fucntions in the GUI
        prismGroupLayout = QVBoxLayout()
        prismGroup = QGroupBox('Robot')

        # Add dropdown for Robot_Base type
        self.baseOption = QComboBox()
        l = []
        for key in AVAILABLE_BASE:
            l.append(key)
        self.baseOption.addItems(l)    
        prismGroupLayout.addWidget(self.baseOption)
        self.baseOption.currentIndexChanged.connect(self._current_base)    

        # Add buttons and dropdowns for each prism
        group_label = 'Robot'
        # Creates outer key value as an empty dictionary
        self.buttons[group_label]={}
        self.prismOptions[group_label]={}
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox(group_label, 'Left'))
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox(group_label, 'Top'))
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox(group_label, 'Right'))

        # Create a button to call Horn's Method
        self.btnSolveRobot = QPushButton('Calculate R->L TF')
        self.btnSolveRobot.clicked.connect(partial(self.Solve_onclick, 'Robot'))
        prismGroupLayout.addWidget(self.btnSolveRobot)
        
        # reset robot
        self.btnResetRobot = QPushButton('Reset R->L TF')
        self.btnResetRobot.clicked.connect(partial(self.Reset_onclick, 'Robot'))
        prismGroupLayout.addWidget(self.btnResetRobot)

        # Layout the robot grou in the GUI
        prismGroup.setLayout(prismGroupLayout)
        self.layout.addWidget(prismGroup)
    
    def _createRobotTarget(self):
        # Creating the target robot functions in the GUI for passing TFs
        robotGroupLayout = QHBoxLayout()
        robotGroup = QGroupBox('Target Robot')

        # Creating a button for calculating full transform
        self.button = QPushButton('Calculate')
        robotGroupLayout.addWidget(self.button)

        # Creating a combo box with a list of potential robots
        self.robotOption = QComboBox()
        self.robotOption.addItems(AVAILABLE_ROBOTS)
        self.robotOption.currentIndexChanged.connect(self._current_robot)
        robotGroupLayout.addWidget(self.robotOption)

        # Creating a sendTF button
        self.sendtfbtn = QPushButton('Send TF')
        self.sendtfbtn.clicked.connect(self._sendTF)    
        robotGroupLayout.addWidget(self.sendtfbtn)

        # Adding layout to GUI
        robotGroup.setLayout(robotGroupLayout)
        self.layout.addWidget(robotGroup)

    #################################################################
    # Button connections and general functions here
    #################################################################
    def _sendTF(self):
        l = 1
    def _calcTF(self):
        global project_transform_from_3d_to_2d
        self.Transform_robot_gate = tf.transformations.concatenate_matrices(self.Transform_robot_leica,tf.transformations.inverse_matrix(self.Transform_gate_leica))
        if not self.Trg_found:
            rospy.loginfo("Robot->Gate:\n%s, %s",\
                tf.transformations.translation_from_matrix(self.Transform_robot_gate).__str__(),\
                [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(self.Transform_robot_gate, 'sxyz')].__str__())
            if project_transform_from_3d_to_2d:
                rospy.loginfo("Projecting 3d transfrom to x-y plane...")
                yaw, pitch, roll = tf.transformations.euler_from_matrix(self.Transform_robot_gate[0:3,0:3], axes="szyx")
                R = tf.transformations.euler_matrix(yaw, 0.0, 0.0, axes="szyx")
                self.Transform_robot_gate[0:3, 0:3] = R[0:3, 0:3]
                rospy.loginfo("New (yaw, pitch, roll) = (%0.4f, %0.4f, %0.4f)" % (yaw*180.0/np.pi, 0.0, 0.0))
        self.Trg_found = True

    def _current_gate(self):
        global V_gate_prism, current_gate

        # Use the current value of gate and base to chose proper value for gate prisms for comparison
        current_gate = self.gateOption.currentText()
        V_gate_prism = [AVAILABLE_GATES[current_gate][key] for key in AVAILABLE_GATES[current_gate]]
        print current_gate
        print V_gate_prism

    def _current_base(self):
        global V_robot_prism, current_base
    
        # Use the current value of gate and base to chose proper value for gates and base prisms for comparison
        current_base = self.baseOption.currentText()
        V_robot_prism = [AVAILABLE_BASE[current_base][key] for key in AVAILABLE_BASE[current_base]]
        print current_base
        print V_robot_prism

    def _current_robot(self):
        global robot_ns
        robot_ns = self.robotOption.currentText()
        print robot_ns

    def _createExit(self):
        #Exit Button layout
        self.btnQuit = QPushButton('Exit')
        self.btnQuit.clicked.connect(self.btnQuit_onclick)
        self.layout.addWidget(self.btnQuit)

    def btnQuit_onclick(self):
        # self.pub_thread.join()
        self.parent().close()

    def _connectSignals(self):
        for group_label in self.buttons:
            for prism_label, button in self.buttons[group_label].items():
                button.clicked.connect(partial(self.find_location,group_label,prism_label))
    
    def find_location(self,group_label,prism_label):
        global V_leica_prism_gate, V_leica_prism_robot

        # For the correct group label and prism label find the TF
        rospy.loginfo("Calculating %s %s location", group_label, prism_label)
        print group_label, prism_label
        pos = self.getTFOnClick(self.buttons[group_label][prism_label],self.prismOptions[group_label][prism_label].currentText())
        
        # Apply pose to the proper group, gate or robot
        if group_label == 'Gate':
            # Check that the returned pose isn't the default pose
            if not all([i==0] for i in pos):
                V_leica_prism_gate[self.point_from_label(prism_label)] = pos

            # Apply an offset based on the height of prisms
            delta_z = AVAILABLE_PRISMS[prism_label]["z"]
            if isinstance(delta_z, list):
                ropsy.logerror("Multiple available prisms of same name.")
                return pos
            V_gate_prism[self.point_from_label(prism_label)][2] += delta_z

        elif group_label == 'Robot':
            # Check that the returned pose isn't the default pose
            if not all([i==0] for i in pos):
                V_leica_prism_robot[self.point_from_label(prism_label)] = pos
        
            # Apply an offset based on the height of prisms
            delta_z = AVAILABLE_PRISMS[prism_label]["z"]
            if isinstance(delta_z, list):
                ropsy.logerror("Multiple available prisms of same name.")
                return pos
            V_robot_prism[self.point_from_label(prism_label)][2] += delta_z
        
        else:
            rospy.logerror("Invalid Group Label")
            return pos

    def point_from_label(self,label):
        point = {
            'Left':1,
            'Top':2,
            'Right':3
            }
        return point.get(label)

    def getTFOnClick(self,prism_btn,prism_name):
        pos = [0,0,0]
           
        prism_btn.setEnabled(False)
        # set prism type in Leica to that which is currently displayed in the GUI
        LS.LeicaSetPrismType(prism_name)

        got_pos = False
        no_fails = 0
        max_no_fails = 5
        while not got_pos:
            LS.LeicaStartTracking()
            pos = LS.LeicaGetPos()
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
            LS.LeicaStopTracking()
        return pos

    def Reset_onclick(self, group_label):
        global V_gate_prism, V_leica_prism_gate, V_leica_prism_robot, V_robot_prism
        if group_label == 'Gate':
            rospy.loginfo("Resetting Gate")
            LS.LeicaStopTracking()

            V_leica_prism_gate = [[None]*3 for i in range(3)]
            self.Trg = None

            self.Tgl_found = False
            self.Trg_found = False

            for prism_label, button in self.buttons[group_label].items():
                button.setEnabled(True)
        elif group_label == 'Robot':
            rospy.loginfo("Resetting Robot")
            LS.LeicaStopTracking()

            V_leica_prism_robot = [[None]*3 for i in range(3)]
            self.Trg = None

            self.Tgl_found = False
            self.Trg_found = False

            for prism_label, button in self.buttons[group_label].items():
                button.setEnabled(True)
        else:
            rospy.logerror("Invalid Group Label, Reset Failed")
            return -1

    def Solve_onclick(self,group_label):
        global V_gate_prism, V_leica_prism_gate, V_leica_prism_robot, V_robot_prism
        if group_label == 'Gate':
            rospy.loginfo("Calculating Gate->Leica")
            if not any(None in pt for pt in V_leica_prism_gate):
                self.Transform_gate_leica = tf_calc.solveForT(V_gate_prism,V_leica_prism_gate)
                rospy.loginfo("Gate->Leica:\n%s, %s",\
                    tf.transformations.translation_from_matrix(self.Transform_gate_leica).__str__(),\
                    [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(Tgl, 'sxyz')].__str__())
            else:
                rospy.logwarn("Three prisms needed")
        elif group_label == 'Robot':
            rospy.loginfo("Calculaing Robot->Leica")
            if not any(None in pt for pt in V_leica_prism_robot):
                self.Transform_robot_leica = tf_calc.solveForT(V_robot_prism,V_leica_prism_robot)
                rospy.loginfo("Robot->Leica:\n%s, %s",\
                    tf.transformations.translation_from_matrix(self.Transform_robot_leica).__str__(),\
                    [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(Trl, 'sxyz')].__str__())
            else:
                rospy.logwarn("Three prisms needed")
        else:
           rospy.logerror("Invalid Group Label, Solve Failed")
           return -1 
    
    ################################################################################
    # End of PrismMonitorWidget Class
    ################################################################################

def main():
    rospy.init_node('prism_monitor_node')
    app = QApplication(sys.argv)
    mainWidget = PrismMonitorWidget(app)
    mainWindow = QMainWindow()
    mainWindow.setWindowTitle('Prism Position Tracker')
    mainWindow.setCentralWidget(mainWidget)
    mainWindow.setStatusBar(QStatusBar())
    mainWindow.show()
    sys.exit(app.exec_())
    

if __name__ == '__main__':
#    main()
    try:
    	main()
    except rospy.ROSInterruptException:
	pass
