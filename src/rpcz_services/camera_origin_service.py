
from gen_proto import get_camera_origin_pb2
from gen_proto import get_camera_origin_rpcz

from base_service import BaseService

import rospy
import numpy as np
import tf_conversions
import tf


class CameraOriginService(get_camera_origin_rpcz.CameraOriginService, BaseService):

    def __init__(self, ros_interface):
        super(CameraOriginService, self).__init__(ros_interface)
        rospy.loginfo("CameraOriginService inited")

    def build_response(self, request):
        rospy.loginfo("CameraOriginService build_response")
        response = get_camera_origin_pb2.CameraOriginResponse()

        camera_origin = self.ros_interface.handle_camera_origin_request()

        if camera_origin:
            response.cameraOrigin.position.x = camera_origin[0]
            response.cameraOrigin.position.y = camera_origin[1]
            response.cameraOrigin.position.z = camera_origin[2]

        return response
