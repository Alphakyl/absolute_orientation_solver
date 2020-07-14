#!usr/bin/python2
import numpy as np
import tf_calc
import tf

robot_ns = "H01"
child_frame_id = "world"
# child_frame_id = "gate_leica"
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

# List of base dictionaries
AVAILABLE_BASE = {
    "UGV": {
        "Vrq1"      : [-0.0713284, 0.0768284, 0.0209],
        "Vrq2"      : [0.0713284, -0.0768284, 0.0209],
        "Vrq3"      : [-0.0713284, -0.0768284, 0.0209],
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
    "oldSmall": {
        "Vgp1"      : [-0.045, 0.100025, 0], # left
        "Vgp2"      : [0.045, -0.100025, 0], # top
        "Vgp3"      : [-0.045, -0.100025, 0], # right
    },
    "Small": {
        "Vgp1"      : [-0.0713284, 0.0768284, 0.0209],
        "Vgp2"      : [0.0713284, -0.0768284, 0.0209],
        "Vgp3"      : [-0.0713284, -0.0768284, 0.0209],
    }
}

# Empty global variables for storing prism points (Given values and calculated from the Leica)
V_gate_prism = [AVAILABLE_GATES[current_gate]['Vgp1'], AVAILABLE_GATES[current_gate]['Vgp2'], AVAILABLE_GATES[current_gate]['Vgp3']]
V_robot_prism = [AVAILABLE_BASE[current_base]['Vrq1'], AVAILABLE_BASE[current_base]['Vrq2'], AVAILABLE_BASE[current_base]['Vrq3']]
V_leica_prism_gate = [[None]*3 for i in range(3)]
V_leica_prism_robot = [[None]*3 for i in range(3)]

# V_gate_prism = [[None]*3 for i in range(3)]

CURRENT_PRISM = {
    "Gate": {
        "Left" : "micro_360",
        "Right": "micro_360",
        "Top": "micro_360",
    },
    "Robot": {
        "Left" : "mini_360",
        "Right": "mini_360",
        "Top": "mini_360",
    }
}



class Test_Prism_Monitor():
    def __init__(self):
        # prism1 = [3.7571, 2.0081, -1.1781] #top
        # prism2 = [3.8469, 1.8526, -1.1732] #left
        # prism3 = [3.7586, 1.8527, -1.175] #right

        # prismr1 = [3.0797, 0.0631, -0.5261] #left
        # prismr2 = [2.9774, 0.1632, -0.5267] #right 
        # prismr3 = [3.0849, 0.2704, -0.5257] #top
        # Transforms found set to false
        # prismr1 = [2.1739, -0.2881, -1.1732]
        # prismr3 = [2.2487, -0.4835, -1.1709]
        # prismr2 = [2.1169, -0.4277, -1.1737]

        # prism2 = [2.097, -0.4801, -1.1683]
        # prism1 = [2.1725, -0.6755, -1.1672]
        # prism3 = [2.039, -0.6266, -1.1702]
        
        ## Robot prism 1m
        # pr1 = [1.2127, 0.7072, -1.1661]
        # pr2 = [1.4105, 0.6411, -1.1642]
        # pr3 = [1.2854, 0.5736, -1.1632]

        ## Gate prism 1m
        # p1 = [1.1122, 0.8919, -1.1648]
        # p2 = [1.3102, 0.8254, -1.1636]
        # p3 = [1.185, 0.7579, -1.1628]

        ## Robot prism 5m
        pr1 = [4.7936, 0.7133, -1.1588]
        pr2 = [4.9039, 0.5273, -1.1567]
        pr3 = [4.7644, 0.5646, -1.1601]

        ## Gate prism 5m
        p1 = [4.834, 0.9204, -1.1524]
        p2 = [4.945, 0.7433, -1.1495]
        p3 = [4.8056, 0.7702, -1.1546]


        self.Trg_found = False
        self.Trl_found = False
        self.Tgl_found = False
        
        # Store empty transforms
        self.Transform_robot_gate = None
        self.Transform_gate_leica = None
        self.Transform_robot_leica = None
        
        # Establish prism adds
        
        # self.find_location('Gate','Top', prism1)
        # self.find_location('Gate','Left', prism2)
        # self.find_location('Gate','Right', prism3)

        # V_gate_prism[0] = pr1
        # V_gate_prism[1] = pr2 
        # V_gate_prism[2] = pr3

        self.find_location('Gate', 'Left', p1)
        self.find_location('Gate', 'Top', p2)
        self.find_location('Gate', 'Right', p3) 

        self.Solve_onclick('Gate')

        # self.find_location('Robot','Right', prismr2)
        # self.find_location('Robot','Left', prismr1)
        # self.find_location('Robot','Top', prismr3)
        self.find_location('Robot', 'Left', pr1)
        self.find_location('Robot', 'Top', pr2)
        self.find_location('Robot', 'Right', pr3)

        self.Solve_onclick('Robot')

        self._calcTF()

    def find_location(self, group_label, prism_label, pos):
        global V_leica_prism_gate, V_leica_prism_robot
        print group_label, prism_label
        
        # Apply pose to the proper group, gate or robot
        if group_label == 'Gate':
            # Check that the returned pose isn't the default pose
            # print pos
            if not all([i==0 for i in pos]):
                V_leica_prism_gate[self.point_from_label(prism_label)] = pos

            # Apply an offset based on the height of prisms
            ## TODO: Potential bug, this never gets reset, make part of prism toggle?
            # delta_z = AVAILABLE_PRISMS[CURRENT_PRISM[group_label][prism_label]]["z"]
            # if isinstance(delta_z, list):
            #     print "Multiple available prisms of same name."
            #     return pos
            # V_gate_prism[self.point_from_label(prism_label)][2] += delta_z

        elif group_label == 'Robot':
            # Check that the returned pose isn't the default pose
            if not all([i==0 for i in pos]):
                V_leica_prism_robot[self.point_from_label(prism_label)] = pos
        
            # Apply an offset based on the height of prisms
            ## TODO: Potential bug, this never gets reset, make part of prism toggle?
            # delta_z = AVAILABLE_PRISMS[CURRENT_PRISM[group_label][prism_label]]["z"]
            # if isinstance(delta_z, list):
            #     print "Multiple available prisms of same name."
            #     return pos
            # V_robot_prism[self.point_from_label(prism_label)][2] += delta_z
        
        else:
            print "Invalid Group Label"
            return pos

    def point_from_label(self,label):
        # Creates a simple switch statement dictionary
        point = {
            'Left':0,
            'Top':1,
            'Right':2,
            }
        return point.get(label)
    
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
            # print "New (yaw, pitch, roll) = (%0.4f, %0.4f, %0.4f)" % (yaw*180.0/np.pi, 0.0, 0.0))
        # Mark TF as found
        self.Trg_found = True


def main():
    test = Test_Prism_Monitor()

if __name__ == '__main__':
    main()