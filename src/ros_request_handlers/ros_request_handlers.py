
#these classes all receive ros service, or action requests, and return ros responses.

#for camera origin
import rospy
import numpy as np
import tf_conversions
import tf

#for reachability
from moveit_trajectory_planner.srv import *

#for execute grasp
import graspit_msgs.msg
import graspit_msgs.srv

#for recognition request
import std_msgs
from std_srvs.srv import Empty
from model_rec2.srv import *


class CameraOriginRequestHandler():

    def __init__(self):
        self._transform_listener = tf.TransformListener()

    def handle(self):

        world_transform = self._get_world_transform()

        camera_origin = None

        rospy.loginfo("world transform: " + str(world_transform))

        if world_transform != None:
            camera_origin = np.linalg.inv(world_transform)[0:3, 3]

        rospy.loginfo("camera_origin: " + str(camera_origin))
        return camera_origin

    def _get_world_transform(self):
        try:
            self._transform_listener.waitForTransform("/camera_rgb_optical_frame", "/world", rospy.Time(0), rospy.Duration(10))
        except:
            rospy.logwarn("Failed to get world transform for: " + self.__class__.__name())
            return None
        world_transform = tf_conversions.toMatrix(tf_conversions.fromTf(
            self._transform_listener.lookupTransform(
                "/camera_rgb_optical_frame", '/world', rospy.Time(0))))
        return world_transform


class ObjectRecognitionRequestHandler():

    def __init__(self):
        #rospy.wait_for_service('recognize_objects', timeout=5)
        #self.recognize_objects_proxy = rospy.ServiceProxy('recognize_objects', FindObjects)
        rospy.wait_for_service("/get_object_info", timeout=10)
        rospy.wait_for_service("model_manager/refresh_model_list", timeout=5)
        self.refresh_model_list_service = rospy.ServiceProxy("model_manager/refresh_model_list", Empty)
        self.get_object_info_service = rospy.ServiceProxy("/get_object_info", graspit_msgs.srv.GetObjectInfo)

    def handle(self):
        self.refresh_model_list_service()
        return self.get_object_info_service()


class CheckReachabilityRequestHandler():

    def __init__(self):
        rospy.wait_for_service('moveit_trajectory_planner/check_reachability', timeout=5)
        self.check_reachability_proxy = rospy.ServiceProxy('moveit_trajectory_planner/check_reachability', LocationInfo)

    def handle(self, ros_request):
        check_reachability_ros_response = self.check_reachability_proxy(ros_request)
        return check_reachability_ros_response


class ExecuteGraspRequestHandler():

    def __init__(self):
        self.grasp_pub = rospy.Publisher('/graspit/grasps', graspit_msgs.msg.Grasp)

    def handle(self, ros_grasp_msg):
        self.grasp_pub.publish(ros_grasp_msg)
