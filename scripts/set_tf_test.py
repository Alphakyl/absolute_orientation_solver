#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

from future import standard_library 
standard_library.install_aliases()
from builtins import str, object

import rospy
from geometry_msgs.msg import *
from marble_origin_detection_msgs.srv import *

class SetTFTest():
    def __init__(self,robot_ns):
        self.trackingSvc = rospy.Service("/"+robot_ns+"/"+"set_world_tf", SetTF, self.svcSetTF)
        rospy.loginfo(robot_ns+" SetTF test started")
    
    def svcSetTF(self, req):
        rospy.loginfo("Got TF:\n%s",req.transform)
        res = SetTFResponse()
        res.success = True
        res.message = "OK"
        return res
        
if __name__ == '__main__':
    robot_ns = sys.argv[1]
    rospy.init_node(robot_ns+"_set_tf_test")
    set_tf_test = SetTFTest(robot_ns)
    rospy.spin()
