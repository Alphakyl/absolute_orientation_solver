#!/usr/bin/python2
import numpy as np
import rospy
import tf

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
    c1 = np.sum(v1,1)/np.size(v1,1)
    c1 = c1[:,np.newaxis]
    print "c1 = " + c1.__str__()
    c2 = np.sum(v2,1)/np.size(v2,1)
    c2 = c2[:,np.newaxis]
    print "c2 = " + c2.__str__()

    # Update coordinates by removing their centroids
    v1_prime = v1-c1
    print "v1_prime = " + v1_prime.__str__()
    v2_prime = v2-c2
    print "v2_prime = " + v2_prime.__str__()

    # Determine the scale factor = sqrt(sum ||v2,i||^2/sum||v1,i||^2)
    S1 = np.sum([np.dot(v1_prime[:,col],v1_prime[:,col]) for col in range(np.size(v1,1))])
    S2 = np.sum([np.dot(v2_prime[:,col],v2_prime[:,col]) for col in range(np.size(v2,1))])
    s = np.sqrt(S2/S1)
    print "scale = " + s.__str__()

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
    quat_mat = tf.transformations.quaternion_matrix([rot[1], rot[2], rot[3], rot[0]])
    
    # Solve for translation based on difference of transformed centroids
    c1 = np.append(c1,np.zeros((1,np.size(c1,1))),axis=0)
    c2 = np.append(c2,np.zeros((1,np.size(c2,1))),axis=0)
    trans = c2-s*np.dot(quat_mat,c1)
    trans = [trans[0],trans[1],trans[2]]
    trans_mat = tf.transformations.translation_matrix(trans)

    # Full solution from rotation and translation matrix
    solution = np.dot(trans_mat,quat_mat)

    # Solve for residuals and error
    v1 = np.append(v1,np.zeros((1,np.size(v1,1))), axis=0)
    v2 = np.append(v2,np.zeros((1,np.size(v2,1))), axis=0)
    residuals = v2-np.dot(solution,v1)
    error = np.sum([np.dot(residuals[:,col],residuals[:,col]) for col in range(np.size(v1,1))])
    
    # Return error and solution
    return error, solution

def solveForT(v1,v2):
    # Minimize using horns method
    error, solution = horns_method(v1,v2)
    print "Calculated Error:", error
    return solution

# def calcTF():   #Robot_origin->World Publisher
#         global Vgp,Vlp,Vrq,Vlq,Tgr

#         if len(Vgp)<3 or len(Vrq)<3:
#             rospy.logerror("Invalid ground truth parameters")
#             return

#         while not rospy.is_shutdown():
#             if self.need_to_reset_gate:
#                 self.resetGate()
#             if self.need_to_reset_robot:
#                 self.resetRobot()
#             if not any(None in pt for pt in Vlp) and not self.Tgl_found:
#                 Tgl = self.solveForT(Vgp,Vlp)
#                 rospy.loginfo("Gate->Leica:\n%s, %s",\
#                     tf.transformations.translation_from_matrix(Tgl).__str__(),\
#                     [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(Tgl, 'sxyz')].__str__())
#                 self.Tgl_found = True
#             if not any(None in pt for pt in Vlq) and not self.Trl_found:
#                 Trl = self.solveForT(Vrq,Vlq)
#                 rospy.loginfo("Robot->Leica:\n%s, %s",\
#                     tf.transformations.translation_from_matrix(Trl).__str__(),\
#                     [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(Trl, 'sxyz')].__str__())
#                 self.Trl_found = True
#             if self.Tgl_found and self.Trl_found:
#                 Tgr = tf.transformations.concatenate_matrices(Tgl,tf.transformations.inverse_matrix(Trl))
#                 Trg = tf.transformations.inverse_matrix(Tgr)
#                 self.Trg = Trg
#                 if not self.Trg_found:
#                     rospy.loginfo("Robot->Gate:\n%s, %s",\
#                         tf.transformations.translation_from_matrix(Trg).__str__(),\
#                         [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(Trg, 'sxyz')].__str__())
#                     if kyle_2d:
#                         rospy.loginfo("Projecting 3d transfrom to x-y plane...")
#                         yaw, pitch, roll = tf.transformations.euler_from_matrix(Trg[0:3,0:3], axes="szyx")
#                         R = tf.transformations.euler_matrix(yaw, 0.0, 0.0, axes="szyx")
#                         Trg[0:3, 0:3] = R[0:3, 0:3]
#                         rospy.loginfo("New (yaw, pitch, roll) = (%0.4f, %0.4f, %0.4f)" % (yaw*180.0/np.pi, 0.0, 0.0))
#                         self.Trg = Trg
#                 self.Trg_found = True
#             time.sleep(1)