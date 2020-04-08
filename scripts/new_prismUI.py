#!/usr/bin/python2
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
base_name = "ugv"
gate_name = "alpha"
kyle_2d = True

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

class PrismMonitorWidget(QMainWindow):
    def __init__(self,parent = None):
        super(PrismMonitorWidget,self).__init__()
        self.layout = QVBoxLayout()
        self._centralWidget=QWidget(self)
        self.setCentralWidget(self._centralWidget)
        self._centralWidget.setLayout(self.layout)
        self._createGateCalculate()
        self._createRobotCalculate()
        self._createRobotTarget()
        self._createExit()
        # launch publisher thread
        self.pub_thread = threading.Thread(target=self._calcTF, args=())
        self.pub_thread.start()

    def _createButtonAndPrismComboBox(self,box_label):
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox(box_label)

        self.button = QPushButton('Calculate')
        boxLayout.addWidget(self.button)

        self.prismOption = QComboBox()
        l = []
        for i in range(len(AVAILABLE_PRISMS)):
            l.append(AVAILABLE_PRISMS[i]["name"])
        self.prismOption.addItems(l)
        boxLayout.addWidget(self.prismOption)
        groupBox.setLayout(boxLayout)
        return groupBox
    
    def _createGateCalculate(self):
        # gate group
        prismGroupLayout = QVBoxLayout()
        prismGroup = QGroupBox('Gate') 
        
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox('Left'))
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox('Top'))
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox('Right'))

        # solve gate
        self.btnSolveGate = QPushButton('Calculate G->L TF')
        # self.btnSolveGate.clicked.connect(self.btnSolveGate_onclick)
        prismGroupLayout.addWidget(self.btnSolveGate)

        # reset gate
        self.btnResetGate = QPushButton('Reset G->L TF')
        # self.btnResetGate.clicked.connect(self.btnResetGate_onclick)
        prismGroupLayout.addWidget(self.btnResetGate)

        # gate group
        prismGroup.setLayout(prismGroupLayout)
        self.layout.addWidget(prismGroup)

    def _createRobotCalculate(self):
        # robot group
        prismGroupLayout = QVBoxLayout()
        prismGroup = QGroupBox('Robot')
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox('Left'))
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox('Top'))
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox('Right'))

        # solve robot
        self.btnSolveRobot = QPushButton('Calculate R->L TF')
        # self.btnSolveRobot.clicked.connect(self.btnSolveRobot_onclick)
        prismGroupLayout.addWidget(self.btnSolveRobot)
        
        # reset robot
        self.btnResetRobot = QPushButton('Reset R->L TF')
        # self.btnResetRobot.clicked.connect(self.btnResetRobot_onclick)
        prismGroupLayout.addWidget(self.btnResetRobot)

        # robot group
        prismGroup.setLayout(prismGroupLayout)
        self.layout.addWidget(prismGroup)
    
    def _createRobotTarget(self):
        robotGroupLayout = QHBoxLayout()
        robotGroup = QGroupBox('Target Robot')
        self.button = QPushButton('Calculate')
        robotGroupLayout.addWidget(self.button)
        self.robotOption = QComboBox()
        self.robotOption.addItems(AVAILABLE_ROBOTS)
        robotGroupLayout.addWidget(self.robotOption)
        self.sendtfbtn = QPushButton('Send TF')
        self.sendtfbtn.clicked.connect(self._sendTF) 
        robotGroupLayout.addWidget(self.sendtfbtn)
        robotGroup.setLayout(robotGroupLayout)
        self.layout.addWidget(robotGroup)

    def _sendTF(self):
        l = 1
    def _calcTF(self):
        l = 1

    def _createExit(self):
        #Exit Button layout
        self.btnQuit = QPushButton('Exit')
        self.btnQuit.clicked.connect(self.btnQuit_onclick)
        self.layout.addWidget(self.btnQuit)

    def btnQuit_onclick(self):
        self.pub_thread.join()
        self.parent().close()

        

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
    try:
	main()
    except rospy.ROSInterruptException:
	pass
