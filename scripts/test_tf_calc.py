# Libraries
#from prismUI import AVAILABLE_GATES
try:
    from tf import transformations
except ImportError:
    import sys
    sys.path.append('../../tf/src/tf')
    import transformations


import math 
import random as rd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

#from tf_calc import solveForT

# Todo:
# Transform to roll, pitch, yaw, x, y, z - Done
# Compare to original
# % error on rotation - care more about rotation - 0.3 degress
# % error translation -
# 

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
    tras_t = [trans[0], trans[1],trans[2] ] # add a vector
    trans_mat = transformations.translation_matrix(tras_t)

    # Full solution from rotation and translation matrix
    solution = np.dot(trans_mat,quat_mat)

    # Solve for residuals and error
    v1 = np.append(v1,np.zeros((1,np.size(v1,1))), axis=0)
    v2 = np.append(v2,np.zeros((1,np.size(v2,1))), axis=0)
    residuals = v2-np.dot(solution,v1)
    error = np.sum( [ math.sqrt( np.dot(residuals[:,col] , residuals[:,col] )) for col in range(np.size(v1,1))])
    
    # Return error and solution
    return error, solution
################################## End ##################################

######################## Test Code 2 ###########################

def dcm2ypr(R):
    # Takes matrix and converted to roll pitch yaw

    yaw = math.atan2(R[0][1], R[0][0])*180.0/math.pi
    pitch = math.asin(-R[0][2])*180.0/math.pi
    roll = math.atan2(R[1][2], R[2][2])*180.0/math.pi

    return roll, pitch, yaw 

def v1tov2(v1, phi, theta, psi, M_transl):
    phi = phi * math.pi/180.0 # roll
    theta = theta * math.pi/180.0# pitch
    psi = psi * math.pi/180.0 #yaw

    RT = np.identity(4)

    RT[0][0] = math.cos(theta)*math.cos(psi) # r11
    RT[0][1] = math.cos(theta)*math.sin(psi) # r12
    RT[0][2] = -math.sin(theta) # r13
    RT[1][0] = math.sin(phi)*math.sin(theta)*math.cos(psi) - math.cos(phi)*math.sin(psi) # r21
    RT[1][1] = math.sin(phi)*math.sin(theta)*math.sin(psi) + math.cos(phi)*math.cos(psi) #r22
    RT[1][2] = math.sin(phi)*math.cos(theta) # r23
    RT[2][0] = math.cos(phi)*math.sin(theta)*math.cos(psi) + math.sin(phi)*math.sin(psi) # r31
    RT[2][1] = math.cos(phi)*math.sin(theta)*math.cos(psi) - math.sin(phi)*math.cos(psi) # r32
    RT[2][2] = math.cos(phi)*math.cos(theta) # r33
    RT[0][3] = M_transl[0] # tx
    RT[1][3] = M_transl[1] # ty
    RT[2][3] = M_transl[2] # tz

    v2 = RT.dot(v1)

    return v2, RT

def rotationMatrix(phi, theta, psi, magnitude_vector):
    v1 = np.zeros((3,4))
    v2 = np.zeros((3,4))

    for i in range(1,4):
        # extracting vector
        v1_temp = np.array([AVAILABLE_GATES[0]["Vgp"+str(i)]])
        #v1_temp = np.array([AVAILABLE_BASE[0]["Vrq"+str(i)]])
        v1_p = np.append(v1_temp,1)
        v1[i-1][:] = v1_p

        # Calculate only one matrix
        if i == 1:
            x = rd.random()
            y = rd.uniform(0,0.15)
            z = 1-x-y
            translation_vector = magnitude_vector*np.array([x,y,z])

            v2_p, RT = v1tov2(v1_p, phi, theta, psi, translation_vector)
        # Since the matrix is already computed just do the dot product
        else:
            v2_p = RT.dot(v1_p)

        v2[i-1][:] = v2_p


    v1 = v1[:,:-1]
    v2 = v2[:,:-1]

    return v1, v2, RT

        # run horns methods    
        

################################# End ############################

def testHorn_Distance():
    phi = 0.3  # roll
    theta = 1.1 # pitch
    psi = 4.9 #yaw
    err = []

    #mag_vector = 10
    for mag_vector in range(5,10):

        
        
        error, R =  horns_method(v1,v2)
        '''
        print("Trial", mag_vector)
        print ("Actual: ", RT)
        print ("Horns: ", R)
        print("Diff: ", R-RT,"\n")
        '''
        print (dcm2ypr(R))
        print (dcm2ypr(RT))

        err.append(error)
        

    plt.plot(err)
    plt.xlabel("Distance Magnitude")
    plt.ylabel("%% Error")
    plt.title("Roll: %i , Pitch: %i, Yaw:%i"%(phi,theta,psi))
    plt.show()

def testHorn_Yaw():
    phi = 1  # roll
    theta = 1 # pitch
    mag_vector = 5 # m
    err = []

    for psi in range(0,20):

        v1,v2,RT = rotationMatrix(phi, theta, psi, mag_vector)

        # run horns methods    
        error, R =  horns_method(v1,v2)
        print (dcm2ypr(R))
        print (dcm2ypr(RT))
        err.append(error)
        #print RT, "\n"
        #print R

        print ("Error", error)

    plt.plot(err)
    plt.xlabel("Yaw")
    plt.ylabel("%% Error")
    plt.title("Roll: %i , Pitch: %i, Magnitude: %i "%(phi,theta,mag_vector))
    plt.show()

def testHorn_all():
    
    itera = 10000
    PH = np.zeros((itera))
    TH = np.zeros((itera))
    PS = np.zeros((itera))
    X = np.zeros((itera))
    Y = np.zeros((itera))
    Z = np.zeros((itera))

    for i in range(0,itera):
        phi = rd.uniform(-0.5,0.5)
        theta = rd.uniform(-0.5,0.5)
        psi = rd.uniform(-45,45)# Yaw
        mag_vector = rd.uniform(5,10)# Yaw

        v1,v2, R_m = rotationMatrix(phi, theta, psi, mag_vector)

        # run horns methods    
        error, R_h =  horns_method(v1,v2)

        print("iteration #: ", i)
        #print(R_m,"\n")
        #print(R_h)
        temp_m = 0
        temp_h = 0
        for j in range(0,2):
            temp_m += R_m[j][3]**2 
            temp_h += R_h[j][3]**2

        mag_m = math.sqrt(temp_m)
        mag_h = math.sqrt(temp_h)
        #print ("magnitude: ",mag_vector, mag_m, mag_h)

        phi_H, theta_H, psi_H = dcm2ypr(R_h)

        phi_err = (phi_H-phi)
        theta_err = (theta_H-theta)
        psi_err = (psi_H-psi)
        x_err = (R_h[0][3]-R_m[0][3])
        y_err = (R_h[1][3]-R_m[1][3])
        z_err = (R_h[2][3]-R_m[2][3])

        #print("RPY: ", phi,theta,psi)
        #print("RPY horns:, ", phi_H, theta_H, psi_H)
        #print(phi_err, theta_err, psi_err)
        #print(x_err,y_err,z_err,"\n")



        PH[i] = phi_err
        TH[i] = theta_err
        PS[i] = psi_err
        X[i] = x_err
        Y[i] = y_err
        Z[i] = z_err

    # Calculate the mean
    phi_mean = math.sqrt(np.sum(np.dot(PH,PH))/itera)
    theta_mean = math.sqrt(np.sum(np.dot(TH,TH))/itera)
    psi_mean = math.sqrt(np.sum(np.dot(PS,PS))/itera)
    X_mean = math.sqrt(np.sum(np.dot(X,X))/itera)
    Y_mean = math.sqrt(np.sum(np.dot(Y,Y))/itera)
    Z_mean = math.sqrt(np.sum(np.dot(Z,Z))/itera)

    print(phi_mean, theta_mean, psi_mean, X_mean, Y_mean, Z_mean)

    # calculate standard diviation
    phi_std = math.sqrt(np.sum(np.dot((PH-phi_mean),(PH-phi_mean))/itera))
    theta_std = math.sqrt(np.sum(np.dot((TH-theta_mean),(TH-theta_mean))/itera))
    psi_std = math.sqrt(np.sum(np.dot((PS-psi_mean),(PS-psi_mean))/itera))
    X_std = math.sqrt(np.sum(np.dot((X-X_mean),(X-X_mean))/itera))
    Y_std = math.sqrt(np.sum(np.dot((Y-Y_mean),(Y-Y_mean))/itera))
    Z_std = math.sqrt(np.sum(np.dot((Z-Z_mean),(Z-Z_mean))/itera))
    print(phi_std,theta_std,psi_std,X_std,Y_std,Z_std)




if __name__ == '__main__':
    #testHorn_Distance()
    #testHorn_Yaw()
    testHorn_all()

