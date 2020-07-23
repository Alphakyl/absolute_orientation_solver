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
import numpy as np 
from geometry_msgs.msg import *
from leica_ros_msgs.srv import *
import time

#import threading

num_scans = 100

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

        # Create numpy arrays to hold prism data
        self.mini_array = np.zeros((1,3))
        print self.mini_array
        self.micro_array = np.zeros((1,3))
        print self.micro_array
        self.mega_array = np.zeros((1,3))
        print self.mega_array

        # Create a holder dictionary for buttons and combo boxes
        self.buttons = {}
        self.prismOptions = {}

        # Define the Gate layout (First part of central widget)
        self._createPrismScan()
        self._createSave()
        self._createExit()

        # Wire up buttons to do various functions
        self._connectSignals()

        
    def _createButtonAndPrismComboBox(self, box_label):
        # Generic button and combo box layout for prism selection

        # Creates a horizontal layout
        boxLayout = QHBoxLayout()
        groupBox = QGroupBox(box_label)

        # Creates a button and adds it to a dictionary of dictionaries for later use
        self.buttons[box_label] = QPushButton('Calculate')
        boxLayout.addWidget(self.buttons[box_label])

        groupBox.setLayout(boxLayout)
        
        # Returns the layout for use outside this function
        return groupBox
    
    def _createPrismScan(self): 
        # Creating the robot fucntions in the GUI
        prismGroupLayout = QVBoxLayout()
        prismGroup = QGroupBox('Robot')

        # Add buttons
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox('Micro'))
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox('Mini'))
        prismGroupLayout.addWidget(self._createButtonAndPrismComboBox('Mega'))
    
        # Layout the robot group in the GUI
        prismGroup.setLayout(prismGroupLayout)
        self.layout.addWidget(prismGroup)
    
    #################################################################
    # Button connections and general functions here
    #################################################################
    def _createSave(self):

        # Creates a button and adds it to a dictionary of dictionaries for later use
        self.save_button = QPushButton('Save')
        self.save_button.clicked.connect(self.btnSave_onclick)
        self.layout.addWidget(self.save_button)

    def _createExit(self):
        #Exit Button layout
        self.btnQuit = QPushButton('Exit')
        self.btnQuit.clicked.connect(self.btnQuit_onclick)
        self.layout.addWidget(self.btnQuit)

    def btnQuit_onclick(self):
        self.parent().close()

    def btnSave_onclick(self):
        print 'saving mini'
        np.savetxt("/home/kyle/catkin_ws/src/absolute_orientation_solver/data/mini" + str(int(time.time())) + ".csv",self.mini_array,delimiter=",")
        print 'saving micro'
        np.savetxt("/home/kyle/catkin_ws/src/absolute_orientation_solver/data/micro" + str(int(time.time())) + ".csv",self.micro_array,delimiter=",")
        print 'saving mega'
        np.savetxt("/home/kyle/catkin_ws/src/absolute_orientation_solver/data/mega" + str(int(time.time())) + ".csv",self.mega_array,delimiter=",")
        print 'done saving'

    def _connectSignals(self):
        # Wires buttons to connection script
        self.buttons['Mini'].clicked.connect(partial(self.find_location,'mini_360'))
        self.buttons['Micro'].clicked.connect(partial(self.find_location,'micro_360'))
        self.buttons['Mega'].clicked.connect(partial(self.find_location,'big_360'))
        
        #for prism_label in self.buttons.items():
        #    self.prism_label.clicked.connect(partial(self.find_location,prism_label))

    def find_location(self,prism_label):
        global num_scans
        print prism_label
        
        # For the correct group label and prism label find the TF
        rospy.loginfo("Calculating %s location", prism_label)
        for i in range(num_scans):
            print i
            pos = self.getTFOnClick(prism_label)
            # Check that the returned pose isn't the default pose
            # print pos
            if not all([i==0 for i in pos]):
                if(prism_label == 'mini_360'):
                    self.mini_array = np.append(self.mini_array,[pos],axis=0)
                elif(prism_label == 'micro_360'):
                    self.micro_array = np.append(self.micro_array,[pos],axis=0)
                elif(prism_label == 'big_360'):
                    self.mega_array = np.append(self.mega_array,[pos],axis=0)
                else:
                    print 'bad prism label' 

    def getTFOnClick(self,prism_name):
        # Create a dummy point
        pos = [0,0,0]

        # set prism type in Leica to that which is currently displayed in the GUI
        LS.LeicaSetPrismType(prism_name)

        # Check if we have the position
        got_pos = False
        no_fails = 0
        max_no_fails = 3
        while not got_pos:
            # Run tracking
            LS.LeicaStartTracking()
            pos = LS.LeicaGetPos()
            got_pos = not all([i==0 for i in pos])
            # If failure
            if not got_pos:
                no_fails += 1
                if no_fails<max_no_fails:
                    rospy.logwarn("Cannot get pos from Leica. Trying again (%d/%d attempts left).",max_no_fails-no_fails,max_no_fails)
                    rospy.logwarn("Possible causes: \n- target prism appears too close to another prism")
                else:
                    rospy.logwarn("Cannot get pos from Leica. Aborting.")
                    got_pos = True
            # End tracking
            LS.LeicaStopTracking()
        return pos
    ################################################################################
    # End of PrismMonitorWidget Class
    ################################################################################

def main():
    # Initialize rosnode
    rospy.init_node('prism_monitor_node')
    # Initiazize application
    app = QApplication(sys.argv)
    # Define the main widget
    mainWidget = PrismMonitorWidget(app)
    # Create a window for the main widgit
    mainWindow = QMainWindow()
    # Give the window a title
    mainWindow.setWindowTitle('Prism Position Tracker')
    # Set the central widet as the main widget
    mainWindow.setCentralWidget(mainWidget)
    # Set up a status bar
    mainWindow.setStatusBar(QStatusBar())
    # Display the main window
    mainWindow.show()
    # Exit on window close
    sys.exit(app.exec_())
    

if __name__ == '__main__':
#    main()
    try:
    	main()
    except rospy.ROSInterruptException:
	pass
