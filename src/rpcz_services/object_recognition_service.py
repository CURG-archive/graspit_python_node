
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

    def build_response(self, request):

        response = run_recognition_pb2.ObjectRecognitionResponse()

        get_object_info_response = self.ros_interface.handle_object_recognition_request()

        for i in range(len(get_object_info_response.object_info)):

            response.foundObjects.add()

            response.foundObjects[i].name = get_object_info_response.object_info[i].object_name
            response.foundObjects[i].pose.position.x = get_object_info_response.object_info[i].object_pose.position.x
            response.foundObjects[i].pose.position.y = get_object_info_response.object_info[i].object_pose.position.y
            response.foundObjects[i].pose.position.z = get_object_info_response.object_info[i].object_pose.position.z
            response.foundObjects[i].pose.orientation.w = get_object_info_response.object_info[i].object_pose.orientation.w
            response.foundObjects[i].pose.orientation.x = get_object_info_response.object_info[i].object_pose.orientation.x
            response.foundObjects[i].pose.orientation.y = get_object_info_response.object_info[i].object_pose.orientation.y
            response.foundObjects[i].pose.orientation.z = get_object_info_response.object_info[i].object_pose.orientation.z

        return response