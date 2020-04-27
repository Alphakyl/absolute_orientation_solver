# Libraries
#from prismUI import AVAILABLE_GATES
try:
    from tf import transformations
except ImportError:
    import sys
    sys.path.append('../../tf/src/tf')
    import transformations

import math 
import numpy as np
from scipy.optimize import minimize

#from tf_calc import solveForT

AVAILABLE_GATES = [
    {
        "name"      : "alpha",
        "Vgp1"      : [0.4425, 1.3275 , 0.844],
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


######################### Horns method ########################
def horns_method(v1,v2):
    # Define pt arrays [[x],[y],...,[z]]
    v1 = np.array(v1).T
    v2 = np.array(v2).T
    
    # Calculate centroids
    c1 = np.sum(v1,1)/np.size(v1,1)
    c1 = c1[:,np.newaxis]
    c2 = np.sum(v2,1)/np.size(v2,1)
    c2 = c2[:,np.newaxis]

    # Update coordinates by removing their centroids
    v1_prime = v1-c1
    v2_prime = v2-c2

    # Determine the scale factor = sqrt(sum ||v2,i||^2/sum||v1,i||^2)
    S1 = np.sum([np.dot(v1_prime[:,col],v1_prime[:,col]) for col in range(np.size(v1,1))])
    S2 = np.sum([np.dot(v2_prime[:,col],v2_prime[:,col]) for col in range(np.size(v2,1))])
    s = np.sqrt(S2/S1)

    # Determine M = [[S_xx S_xy S_xz], [Syx Syy Syz], [Szx Szy Szz]]
    M = np.dot(v1_prime,v2_prime.T)
    S_xx = M[0,0]
    S_xy = M[0,1]
    S_xz = M[0,2]
    S_yx = M[1,0]
    S_yy = M[1,1]
    S_yz = M[1,2]
    S_zx = M[2,0]
    S_zy = M[2,1]
    S_zz = M[2,2]
    N = np.array([
        [S_xx+S_yy+S_zz, S_yz-S_zy, S_zx-S_xz, S_xy-S_yx],
        [S_yz-S_zy, S_xx-S_yy-S_zz, S_xy+S_yx, S_zx+S_xz],
        [S_zx-S_xz, S_xy+S_yx, -S_xx+S_yy-S_zz, S_yz+S_zy],
        [S_xy-S_yx, S_zx+S_xz, S_yz+S_zy, -S_xx-S_yy+S_zz]
    ])
    
    # Rotation quaternion vector is the eigenvector of the most positive eigen value of N
    eig_val,eig_vec = np.linalg.eig(N)
    # rot = [w, x, y, z]
    rot = eig_vec[:,np.argmax(eig_val)]
    quat_mat = transformations.quaternion_matrix([rot[1], rot[2], rot[3], rot[0]]) # change - made them into a vector
    
    # Solve for translation based on difference of transformed centroids
    c1 = np.append(c1,np.zeros((1,np.size(c1,1))),axis=0)
    c2 = np.append(c2,np.zeros((1,np.size(c2,1))),axis=0)
    trans = c2-s*np.dot(quat_mat,c1)
    tras_t = [trans[0], trans[1],trans[2] ] - # add a vector
    trans_mat = transformations.translation_matrix(tras_t)

    # Full solution from rotation and translation matrix
    solution = np.dot(trans_mat,quat_mat)

    # Solve for residuals and error
    v1 = np.append(v1,np.zeros((1,np.size(v1,1))), axis=0)
    v2 = np.append(v2,np.zeros((1,np.size(v2,1))), axis=0)
    residuals = v2-np.dot(solution,v1)
    error = np.sum([np.dot(residuals[:,col],residuals[:,col]) for col in range(np.size(v1,1))])
    
    # Return error and solution
    return error, solution
################################## End ##################################

######################## Test Code 2 ###########################


def v1tov2(v1, phi, theta, psi, M_transl):
    phi = phi * math.pi/180.0 # roll
    theta = theta * math.pi/180.0# pitch
    psi = psi * math.pi/180.0 #yaw

    RT = np.identity(4)

    RT[0][0] = math.cos(theta)*math.cos(psi) # r11
    RT[0][1] =  math.cos(theta)*math.sin(psi) # r12
    RT[0][2] = -math.sin(theta) # r13
    RT[1][0] = math.sin(phi)*math.sin(theta)*math.cos(psi) - math.cos(phi)*math.sin(psi) # r21
    RT[1][1] = math.sin(phi)*math.sin(theta)*math.sin(psi) + math.cos(phi)*math.cos(psi) #r22
    RT[2][1] = math.sin(phi)*math.cos(theta) # r23
    RT[2][0] = math.cos(phi)*math.sin(theta)*math.cos(psi) + math.sin(phi)*math.sin(psi) # r31
    RT[2][1] = math.cos(phi)*math.sin(theta)*math.cos(psi) - math.sin(phi)*math.cos(psi) # r32
    RT[2][2] = math.cos(phi)*math.cos(theta) # r33
    RT[0][3] = M_transl[0] # tx
    RT[1][3] = M_transl[1] # ty
    RT[2][3] = M_transl[2] # tz

    v2 = RT.dot(v1)

    return v2, RT

################################# End ############################

def testHorn():
    phi = 30.0  # roll
    theta = 5.0 # pitch
    psi = -5.0 #yaw
    translation_vector = np.array([-5,-3,-1])

    v1 = np.zeros((3,4))
    v2 = np.zeros((3,4))

    for i in range(1,4):
        # extracting vector
        v1_temp = np.array([AVAILABLE_GATES[0]["Vgp"+str(i)]])
        v1_p = np.append(v1_temp,1)
        v1[i-1][:] = v1_p

        # Calculate only one matrix
        if i == 1:
            v2_p, RT = v1tov2(v1_p, phi, theta, psi, translation_vector)
        # Since the matrix is already computed just do the dot product
        else:
            v2_p = RT.dot(v1_p)

        v2[i-1][:] = v2_p


    v1 = v1[:,:-1]
    v2 = v2[:,:-1]
    print v1
    print v2
    # run horns methods    
    horns_method(v1,v2)



if __name__ == '__main__':
    testHorn()

