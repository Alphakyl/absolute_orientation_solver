#!/usr/bin/env python
import rospy
import roslib
#roslib.load_manifest('ltf')
import math
import tf
from leica_ros.msg import AngleMeasStamped
from  geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import numpy as np  
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
import message_filters
import time



def callback(angCallback, posCallback):
    #Creating a transform broadcaster
    transform_broadcaster  = tf.TransformBroadcaster() 
    #rate = rospy.Rate(1.0)
    #Converting the Azimuth and Zenith angle's leica to Euler - roll,pitch,yaw
    roll = 0
    pitch = math.radians(angCallback.theta)
    yaw = math.radians(angCallback.phi)
    
    #Setting the translation vector 
    translation_vector = (posCallback.point.x,posCallback.point.y,posCallback.point.z) 
  
    #roll-pitch-yaw angles to a quaternion using ROS TF Library
    rotation_quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    #Capturing current time 
    current_time = rospy.Time.now()

    #Broadcasting frame from home to 1st Prism
    transform_broadcaster.sendTransform(translation_vector,rotation_quaternion,current_time,"home","prism")
    #rate.sleep(1.0)
    

    
def main():
    rospy.init_node('leica_tf_broadcaster', anonymous=True )
    while(not rospy.is_shutdown()):
        PrismLowerLeftGate_broadcaster1  = tf.TransformBroadcaster()
        PrismTopGate_broadcaster2  = tf.TransformBroadcaster()
        PrismLowerRightGate_broadcaster3  = tf.TransformBroadcaster()

        RobotLeftPrism_broadcaster1  = tf.TransformBroadcaster()
        RobotTopPrism_broadcaster2  = tf.TransformBroadcaster()
        RobotRightPrism_broadcaster3  = tf.TransformBroadcaster()

        ############## Static Transform from Gate Origin to respective prism ##############
        #create a quaternion
        rotation_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

        #translation vector

        #According to Sheet for older terrain
        # Prism1_translation_vector1 = (0.0,1.592 , 1.11)
        # Prism2_translation_vector2 = (0.0, 0.0, 1.82)
        # Prism3_translation_vector3 = (0.0, -1.671, 0.79)

        Prism1_translation_vector1 = (0.0,1.523 , 1.07)
        Prism2_translation_vector2 = (0.0, 0.0, 1.83)
        Prism3_translation_vector3 = (0.0, -1.394, 0.755)

        ############## Static Transform from Robot_Origin to respective prism ##############
        #create a quaternion
        RobotLeftPrism_rotation_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        RobotTopPrism_rotation_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        RobotRightPrism_rotation_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

        #translation vector
        # RobotLeftPrism_translation_vector1 = (-0.2513, 0.2, 0.2833)
        # RobotTopPrism_translation_vector2 = (0.0013, -0.2, 0.2833)
        # RobotRightPrism_translation_vector3 = (-0.2513, -0.2, 0.2833)
        RobotLeftPrism_translation_vector1 = (-0.045, 0.100025, 0.0174602)
        RobotTopPrism_translation_vector2 = (0.045, -0.100025, 0.0174602)
        RobotRightPrism_translation_vector3 = (-0.045, -0.100025, 0.0174602)

        # #For Updated Prism Rig
        # RobotLeftPrism_translation_vector1 = (0.3929, 0.100025, 0.5247)
        # RobotTopPrism_translation_vector2 = (0.4829, -0.100025, 0.5247)
        # RobotRightPrism_translation_vector3 = (0.3929, -0.100025, 0.5247)

        #time
        current_time = rospy.Time.now()

        PrismLowerLeftGate_broadcaster1.sendTransform(
            Prism1_translation_vector1, 
            rotation_quaternion,
            current_time, 
            "PrismLeftLowerGate", "Gate_Origin") #child frame, parent frame
        PrismTopGate_broadcaster2.sendTransform(
            Prism2_translation_vector2, 
            rotation_quaternion,
            current_time, 
            "PrismTopGate", "Gate_Origin") #child frame, parent frame
        PrismLowerRightGate_broadcaster3.sendTransform(
            Prism3_translation_vector3, 
            rotation_quaternion,
            current_time, 
            "PrismRightLowerGate", "Gate_Origin") #child frame, parent frame
        
        RobotLeftPrism_broadcaster1.sendTransform(
            RobotLeftPrism_translation_vector1, 
            RobotLeftPrism_rotation_quaternion,
            current_time, 
            "RobotLeftPrism", "Robot_Origin") #child frame, parent frame
        RobotTopPrism_broadcaster2.sendTransform(
            RobotTopPrism_translation_vector2, 
            RobotTopPrism_rotation_quaternion,
            current_time, 
            "RobotTopPrism", "Robot_Origin") #child frame, parent frame
        RobotRightPrism_broadcaster3.sendTransform(
            RobotRightPrism_translation_vector3, 
            RobotRightPrism_rotation_quaternion,
            current_time, 
            "RobotRightPrism", "Robot_Origin") #child frame, parent frame
                
        time.sleep(0.5)

    rospy.spin()

if __name__ == '__main__':
    main()
