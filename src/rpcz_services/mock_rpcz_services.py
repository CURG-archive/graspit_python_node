
from gen_proto import run_recognition_pb2
from gen_proto import get_camera_origin_pb2
from gen_proto import check_grasp_reachability_pb2
from gen_proto import execute_grasp_pb2

from gen_proto import run_recognition_rpcz
from gen_proto import get_camera_origin_rpcz
from gen_proto import check_grasp_reachability_rpcz
from gen_proto import execute_grasp_rpcz

import roslib

from rpcz_services.base_service import BaseService

pkg_path = roslib.packages.get_pkg_dir('graspit_python_node')


class MockObjectRecognitionService(run_recognition_rpcz.ObjectRecognitionService, BaseService):

    def build_response(self, request):
        empty_response = run_recognition_pb2.ObjectRecognitionResponse()
        canned_response_filepath = pkg_path + "/src/rpcz_services/canned_proto_responses/object_recognition_response.saved_proto"
        response = _build_response(canned_response_filepath, empty_response)
        return response


class MockCameraOriginService(get_camera_origin_rpcz.CameraOriginService, BaseService):

    def build_response(self, request):
        empty_response = get_camera_origin_pb2.CameraOriginResponse()
        canned_response_filepath = pkg_path + "/src/rpcz_services/canned_proto_responses/camera_origin_response.saved_proto"
        response = _build_response(canned_response_filepath, empty_response)
        return response


class MockCheckGraspReachabilityService(check_grasp_reachability_rpcz.CheckGraspReachabilityService, BaseService):

    def build_response(self, request):
        response = check_grasp_reachability_pb2.CheckGraspReachabilityResponse()
        response.graspId = request.grasp.graspId
        response.graspStatus = True
        return response


class MockExecuteGraspService(execute_grasp_rpcz.ExecuteGraspService, BaseService):

    def build_response(self, request):
        response = execute_grasp_pb2.ExecuteGraspResponse()
        response.isSuccessful = True
        return response


def _build_response(file_path, response):
    f = open(file_path, "rb")
    response.ParseFromString(f.read())
    return response