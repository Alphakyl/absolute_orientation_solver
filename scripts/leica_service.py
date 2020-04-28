import rospy
from leica_ros.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
from geometry_msgs.msg import *


from leica_ros.srv import *
from marble_origin_detection_msgs.srv import *
# from marble_origin_detection_msgs.srv import SetTF, SetTFRequest, SetTFResponse

def LeicaSetPrismType(self,prism_name):
    set_prism_type_svc = rospy.ServiceProxy('leica_node/set_prism_type', SetPrismType)
    try:
        rospy.loginfo("Setting to prism: %s",prism_name)
        set_prism_type_req = SetPrismTypeRequest()
        if prism_name=="micro_360":
            set_prism_type_req.name = "mini_360"
        else:
            set_prism_type_req.name = prism_name
        set_prism_type_resp = set_prism_type_svc(set_prism_type_req)
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s",e)
        return 
    rospy.loginfo("Prism set to %s",prism_name)
    return set_prism_type_resp

def LeicaGetPrismType(self):
    get_prism_type_svc = rospy.ServiceProxy('leica_node/get_prism_type', GetPrismType)
    try:
        rospy.loginfo("Getting current prism name")
        get_prism_type_resp = get_prism_type_svc()
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s",e)
        return 
    current_prism_name = get_prism_type_resp.name
    rospy.loginfo("Current prism: %s",current_prism_name)
    return current_prism_name

def LeicaStartTracking(self):
    start_tracking_svc = rospy.ServiceProxy('leica_node/start_tracking', StartTracking)
    try:
        rospy.loginfo("Starting tracking")
        start_tracking_resp = start_tracking_svc()
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s",e)
        return
    rospy.loginfo("Tracking started")
    return start_tracking_resp

def LeicaStopTracking(self):
    stop_tracking_svc = rospy.ServiceProxy('leica_node/stop_tracking', SetBool)
    try:
        rospy.loginfo("Stopping tracking")
        stop_tracking_req = SetBoolRequest()
        stop_tracking_req.data = False
        stop_tracking_resp = stop_tracking_svc(stop_tracking_req)
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s",e)
        return 
    rospy.loginfo("Tracking stopped")
    return stop_tracking_resp

def LeicaGetPos(self): 
    pos = [0, 0, 0]
    try:
        msg = rospy.wait_for_message("leica_node/position", PointStamped, 5.0)
        pos = [msg.point.x, msg.point.y, msg.point.z]
    except rospy.exceptions.ROSException as e:
        rospy.logwarn("Service call failed: %s",e)
    return pos
