import numpy as np
import rospy
import tf

#!/usr/bin/python2
"""
Horn's Formuala : https://www.mathworks.com/matlabcentral/fileexchange/26186-absolute-orientation-horn-s-method
Horn's Method for Absolute Orientation: https://www.osapublishing.org/josaa/abstract.cfm?uri=josaa-4-4-629 
See Also: Kabsch Algorithm https://en.wikipedia.org/wiki/Kabsch_algorithm#cite_note-1
"""

def horns_method(v1,v2):
    # Define pt arrays [[x],[y],...,[z]]
    v1 = np.array(v1).T
    v2 = np.array(v2).T
    
    # Calculate centroids
    c1 = np.sum(v1,1)/3
    c2 = np.sum(v2,1)/3
    # Update coordinates by removing their centroids
    v1_prime = v1-c1[:,np.newaxis]
    v2_prime = v2-c2[:,np.newaxis]

    # Determine the scale factor = sqrt(sum ||v2,i||^2/sum||v1,i||^2)
    S1 = np.sum([np.dot(v1_prime[:,col],v1_prime[:,col]) for col in range(np.size(v1,1))])
    S2 = np.sum([np.dot(v2_prime[:,col],v2_prime[:,col]) for col in range(np.size(v2,1))])
    s = sqrt(S2/S1)

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
    quat_mat = tf.transformations.quaternion_matrix(rot[1], rot[2], rot[3], rot[0])
    
    # Need to solve for the translation and error
    trans = tf.transformations.translation_matrix(c2+[0]-scale_matrix(s,np.dot(quat_mat,c1+[0])))
    solution = numpy.dot(trans,quat_matrix)
    v1 = [v1+[0] for col in range(np.size(v1,1))]
    v2 = [v2+[0] for col in range(np.size(v2,1))]
    error = np.sum([np.dot(v2[:,col]-scale_matrix(s,np.dot(quat_mat,v1[:,col]))-trans,v2[:,col]-scale_matrix(s,np.dot(quat_mat,v1[:,col]))-trans) for col in range(np.size(v1,1))])
    return error, solution

# def cost_fun(x,v1,v2):
#     v1 = map(list, zip(*v1))
#     v2 = map(list, zip(*v2))
#     # print v1
#     # print v2
#     t_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
#     q_mat = tf.transformations.quaternion_matrix(normalize_quaternion(np.array([x[3], x[4], x[5],x[6]])))
#     mat = np.dot(t_mat, q_mat)
#     err_mat = v1-np.dot(mat,v2)
#     # print err_mat
#     err_norm = LA.norm(err_mat[:,0])+LA.norm(err_mat[:,1])+LA.norm(err_mat[:,2])   
#     return err_norm

# def cost_funz(x,v1,v2):
#     v1 = map(list, zip(*v1))
#     v2 = map(list, zip(*v2))
#     # print v1
#     # print v2
#     t_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
#     q_mat = tf.transformations.quaternion_matrix(normalize_quaternion(np.array([0, 0, x[3], 1-x[3]*x[3]])))
#     mat = np.dot(t_mat, q_mat)
#     err_mat = v1-np.dot(mat,v2)
#     # print err_mat
#     err_norm = LA.norm(err_mat[:,0])+LA.norm(err_mat[:,1])+LA.norm(err_mat[:,2])   
#     return err_norm

def normalize_quaternion(rot):
    ratio = math.sqrt(rot[0]**2 + rot[1]**2 + rot[2]**2 + rot[3]**2)
    return (rot[0]/ratio, rot[1]/ratio, rot[2]/ratio, rot[3]/ratio)

def solveForT(self,v1,v2):
    # Appends 1 to the end of each point
    # v1 = [pt+[1] for pt in v1]
    # v2 = [pt+[1] for pt in v2]
        
    # Minimize using horns method
    error, solution = horns_method(v1,v2)

    # xo = np.array([0,0,0,0,0,0,1])
    # xyzy = np.array([0, 0, 0, 0])
    # solution = minimize(cost_funz,xyzy,method='L-BFGS-B',args=(v1,v2))
    # solution = minimize(cost_fun,[solution.x[0],solution.x[1],solution.x[2],0,0,solution.x[3],(1-solution.x[3]**2)**(1./2)],method='L-BFGS-B',args=(v1,v2))
 
    # Convert xyz difference to matrix
    tran_mat = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
    # Convert rotation quaternion
    quat = np.array([solution.x[3],solution.x[4],solution.x[5],solution.x[6]])
    quat = normalize_quaternion(quat) #Vital for Correct Solution
    # Convert to quaternionr rotation matrix
    quat_mat = tf.transformations.quaternion_matrix(np.array(quat))
    # Set full solution to matrix form
    T12 = tf.transformations.concatenate_matrices(tran_mat,quat_mat)

    return T12

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
                    if kyle_2d:
                        rospy.loginfo("Projecting 3d transfrom to x-y plane...")
                        yaw, pitch, roll = tf.transformations.euler_from_matrix(Trg[0:3,0:3], axes="szyx")
                        R = tf.transformations.euler_matrix(yaw, 0.0, 0.0, axes="szyx")
                        Trg[0:3, 0:3] = R[0:3, 0:3]
                        rospy.loginfo("New (yaw, pitch, roll) = (%0.4f, %0.4f, %0.4f)" % (yaw*180.0/np.pi, 0.0, 0.0))
                        self.Trg = Trg
                self.Trg_found = True
            time.sleep(1)