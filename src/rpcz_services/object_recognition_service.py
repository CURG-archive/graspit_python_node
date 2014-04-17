
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

            graspable_object = graspable_object_pb2.GraspableObject()
            graspable_object.name = find_objects_response.object_name[i]
            graspable_object.pose = find_objects_response.object_pose[i]

            response.foundObjects.add(graspable_object)

        return response