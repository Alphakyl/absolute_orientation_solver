#!/usr/bin/python2
"""
Leica UI node to monitor the prism data

Horn's Formuala : https://www.mathworks.com/matlabcentral/fileexchange/26186-absolute-orientation-horn-s-method
"""
def cost_fun(x,v1,v2):
    v1 = map(list, zip(*v1))
    v2 = map(list, zip(*v2))
    # print v1
    # print v2
    t_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
    q_mat = tf.transformations.quaternion_matrix(normalize_quaternion(np.array([x[3], x[4], x[5],x[6]])))
    mat = np.dot(t_mat, q_mat)
    err_mat = v1-np.dot(mat,v2)
    # print err_mat
    err_norm = LA.norm(err_mat[:,0])+LA.norm(err_mat[:,1])+LA.norm(err_mat[:,2])   
    return err_norm

def cost_funz(x,v1,v2):
    v1 = map(list, zip(*v1))
    v2 = map(list, zip(*v2))
    # print v1
    # print v2
    t_mat = tf.transformations.translation_matrix(np.array([x[0],x[1],x[2]]))
    q_mat = tf.transformations.quaternion_matrix(normalize_quaternion(np.array([0, 0, x[3], 1-x[3]*x[3]])))
    mat = np.dot(t_mat, q_mat)
    err_mat = v1-np.dot(mat,v2)
    # print err_mat
    err_norm = LA.norm(err_mat[:,0])+LA.norm(err_mat[:,1])+LA.norm(err_mat[:,2])   
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

def solveForT(self,v1,v2):
    # v1 = map(list, zip(*v1))
    # v1.append([1,1,1])
    # v2 = map(list, zip(*v2))
    # v2.append([1,1,1])
    v1 = [pt+[1] for pt in v1]
    v2 = [pt+[1] for pt in v2]
    # xo = np.array([0,0,0,0,0,0,1])
    xyzy = np.array([0, 0, 0, 0])
    solution = minimize(cost_funz,xyzy,method='L-BFGS-B',args=(v1,v2))
    solution = minimize(cost_fun,[solution.x[0],solution.x[1],solution.x[2],0,0,solution.x[3],(1-solution.x[3]**2)**(1./2)],method='L-BFGS-B',args=(v1,v2))
    tran_mat = tf.transformations.translation_matrix(np.array([solution.x[0],solution.x[1],solution.x[2]]))
    quat = np.array([solution.x[3],solution.x[4],solution.x[5],solution.x[6]])
    quat = normalize_quaternion(quat) #Vital for Correct Solution
    quat_mat = tf.transformations.quaternion_matrix(np.array(quat))
    T12 = tf.transformations.concatenate_matrices(tran_mat,quat_mat)

    # print v1
    # print v2
    # print T12

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