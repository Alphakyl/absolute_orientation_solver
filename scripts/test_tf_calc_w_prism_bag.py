#!usr/bin/python2
import numpy as np
import tf_calc

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

# List of base dictionaries
AVAILABLE_BASE = {
    "UGV": {
        "Vrq1"      : [-0.0768284,0.0713284,0.0209],
        "Vrq2"      : [0.0768284,-0.0713284,0.0209],
        "Vrq3"      : [-0.0768284,-0.0713284,0.0209],
    },
    "UAV": {
        "Vrq1"      : [-0.25, -.1, -.205],
        "Vrq2"      : [0.25,0.1, -.205],
        "Vrq3"      : [0.25,-.1, -.205],
    },
    "oldUGV": {
        "Vrq1"      : [-0.045, 0.100025, -0.0045],
        "Vrq2"      : [0.045, -0.100025, -0.0045],
        "Vrq3"      : [-0.045, -0.100025, -0.0045],
    }
}

# List of gate dictionaries
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
V_gate_prism = [AVAILABLE_GATES[current_gate]['Vgp1'], AVAILABLE_GATES[current_gate]['Vgp2'], AVAILABLE_GATES[current_gate]['Vgp3']]
V_robot_prism = [AVAILABLE_BASE[current_base]['Vrq1'], AVAILABLE_BASE[current_base]['Vrq2'], AVAILABLE_BASE[current_base]['Vrq3']]
V_leica_prism_gate = [[None]*3 for i in range(3)]
V_leica_prism_robot = [[None]*3 for i in range(3)]

CURRENT_PRISM = {
    "Gate": {
        "Left" : "micro_360",
        "Right": "micro_360",
        "Top": "micro_360",
    },
    "Robot": {
        "Left" : "micro_360",
        "Right": "micro_360",
        "Top": "micro_360",
    }
}



class Test_Prism_Monitor():
    def __init__(self):
        prism1 = [3.7571, 2.0081, -1.1781] #top
        prism2 = [3.8469, 1.8526, -1.1732] #left
        prism3 = [3.7586, 1.8527, -1.175] #right

        prismr1 = [3.0797, 0.0631, -0.5261] #left
        prismr2 = [2.9774, 0.1632, -0.5267] #right 
        prismr3 = [3.0849, 0.2704, -0.5257] #top



    def find_location(self, group_label, prism_label, pos):
        global V_leica_prism_gate, V_leica_prism_robot
        print group_label, prism_label
            pos = self.getTFOnClick(self.buttons[group_label][prism_label],self.prismOptions[group_label][prism_label].currentText())
            
            # Apply pose to the proper group, gate or robot
            if group_label == 'Gate':
                # Check that the returned pose isn't the default pose
                # print pos
                if not all([i==0 for i in pos]):
                    V_leica_prism_gate[self.point_from_label(prism_label)] = pos

                # Apply an offset based on the height of prisms
                delta_z = AVAILABLE_PRISMS[CURRENT_PRISM[group_label][prism_label]]["z"]
                if isinstance(delta_z, list):
                    print "Multiple available prisms of same name."
                    return pos
                V_gate_prism[self.point_from_label(prism_label)][2] += delta_z

            elif group_label == 'Robot':
                # Check that the returned pose isn't the default pose
                if not all([i==0 for i in pos]):
                    V_leica_prism_robot[self.point_from_label(prism_label)] = pos
            
                # Apply an offset based on the height of prisms
                delta_z = AVAILABLE_PRISMS[CURRENT_PRISM[group_label][prism_label]]["z"]
                if isinstance(delta_z, list):
                    print "Multiple available prisms of same name."
                    return pos
                V_robot_prism[self.point_from_label(prism_label)][2] += delta_z
            
            else:
                print "Invalid Group Label"
                return pos

    def Solve_onclick(self,group_label):
            global V_gate_prism, V_leica_prism_gate, V_leica_prism_robot, V_robot_prism
            
            if group_label == 'Gate':
                print "Calculating Gate->Leica"
                # Check that all prisms have been found
                if not any(None in pt for pt in V_leica_prism_gate):
                    # Solve from darpa frame to leica frame
                    self.Transform_gate_leica = tf_calc.solveForT(V_gate_prism,V_leica_prism_gate)
                    print "Gate->Leica:\n"
                    print tf.transformations.translation_from_matrix(self.Transform_gate_leica).__str__() 
                    print [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(self.Transform_gate_leica, 'sxyz')].__str__()
                    # Set transform as found
                    self.Tgl_found = True
                else:
                    print "Three prisms needed"
            
            elif group_label == 'Robot':
                "print Calculaing Robot->Leica"
                if not any(None in pt for pt in V_leica_prism_robot):
                    # Solve from robot frame to leica frame
                    self.Transform_robot_leica = tf_calc.solveForT(V_robot_prism,V_leica_prism_robot)
                    print "Robot->Leica:\ns"
                    print tf.transformations.translation_from_matrix(self.Transform_robot_leica).__str__()
                    print [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(self.Transform_robot_leica, 'sxyz')].__str__()
                    # Set transform as found
                    self.Trl_found = True
                else:
                    print "Three prisms needed"
            
            else:
                print "Invalid Group Label, Solve Failed"
            return -1 
        
        def _calcTF(self):
            global project_transform_from_3d_to_2d

            # Check if gate and robot to leica transforms are available
            if not self.Tgl_found:
                print "Missing Gate->Leica Transform"
                return
            if not self.Trl_found:
                print "Missing Robot->Leica Transform"
                return

            # Trg = Trl*(Tgl)^(-1)
            self.Transform_robot_gate = tf.transformations.concatenate_matrices(self.Transform_robot_leica,tf.transformations.inverse_matrix(self.Transform_gate_leica))

            # If the transform has not been found previously set the transform
            if not self.Trg_found:
                print "Robot->Gate:\n"
                print tf.transformations.translation_from_matrix(self.Transform_robot_gate).__str__()
                print [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(self.Transform_robot_gate, 'sxyz')].__str__()
            # Legacy in case we need to project to 2d in the future
            if project_transform_from_3d_to_2d:
                print "Projecting 3d transfrom to x-y plane..."
                yaw, pitch, roll = tf.transformations.euler_from_matrix(self.Transform_robot_gate[0:3,0:3], axes="szyx")
                R = tf.transformations.euler_matrix(yaw, 0.0, 0.0, axes="szyx")
                self.Transform_robot_gate[0:3, 0:3] = R[0:3, 0:3]
                print "New (yaw, pitch, roll) = (%0.4f, %0.4f, %0.4f)" % (yaw*180.0/np.pi, 0.0, 0.0))
            # Mark TF as found
            self.Trg_found = True