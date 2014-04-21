
from gen_proto import run_recognition_pb2
from gen_proto import run_recognition_rpcz
from gen_proto import graspable_object_pb2

from base_service import BaseService

import std_msgs
import rospy
from model_rec2.srv import *


class ObjectRecognitionService(run_recognition_rpcz.ObjectRecognitionService, BaseService):

    def __init__(self, ros_interface):
        super(ObjectRecognitionService, self).__init__(ros_interface)

    def build_response(self,request):

        response = run_recognition_pb2.ObjectRecognitionResponse()

        find_objects_response = self.ros_interface.handle_object_recognition_request()

        for i in range(len(find_objects_response.object_name)):

            response.foundObjects.add()

            response.foundObjects[i].name = find_objects_response.object_name[i]
            response.foundObjects[i].pose.position.x = find_objects_response.object_pose[i].position.x
            response.foundObjects[i].pose.position.y = find_objects_response.object_pose[i].position.y
            response.foundObjects[i].pose.position.z = find_objects_response.object_pose[i].position.z
            response.foundObjects[i].pose.orientation.w = find_objects_response.object_pose[i].orientation.w
            response.foundObjects[i].pose.orientation.x = find_objects_response.object_pose[i].orientation.x
            response.foundObjects[i].pose.orientation.y = find_objects_response.object_pose[i].orientation.y
            response.foundObjects[i].pose.orientation.z = find_objects_response.object_pose[i].orientation.z

        return response