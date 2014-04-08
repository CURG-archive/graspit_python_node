
from gen_proto import get_camera_origin_pb2
from gen_proto import get_camera_origin_rpcz

from base_service import BaseService

import rospy
import numpy as np
import tf_conversions
import tf


class CameraOriginService(get_camera_origin_rpcz.CameraOriginService, BaseService):

    def __init__(self):
        super().__init__()
        self.transform_listener = tf.TransformListener()

    def build_response(self, request):
        response = get_camera_origin_pb2.CameraOriginResponse()

        world_transform = self.get_world_transform()

        if world_transform:
            origin = np.linalg.inv(world_transform)[0:3, 3]
            response.cameraOrigin.position.x = origin[0]
            response.cameraOrigin.position.y = origin[1]
            response.cameraOrigin.position.z = origin[2]

        return response

    def get_world_transform(self):
        try:
            self.transform_listener.waitForTransform("/camera_rgb_optical_frame", "/world", rospy.Time(0), rospy.Duration(.1))
        except:
            return None
        world_transform = tf_conversions.toMatrix(tf_conversions.fromTf(
            self.transform_listener.lookupTransform(
                "/camera_rgb_optical_frame", '/world', rospy.Time(0))))
        return world_transform