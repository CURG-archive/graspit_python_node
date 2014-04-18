from gen_proto import geometry_msgs_pb2
from gen_proto import graspable_object_pb2
from gen_proto import run_recognition_pb2
from gen_proto import get_camera_origin_pb2
from gen_proto import check_grasp_reachability_pb2
from gen_proto import execute_grasp_pb2

from gen_proto import run_recognition_rpcz
from gen_proto import get_camera_origin_rpcz
from gen_proto import check_grasp_reachability_rpcz
from gen_proto import execute_grasp_rpcz


from ros_services.base_service import BaseService


class MockObjectRecognitionService(run_recognition_rpcz.ObjectRecognitionService, BaseService):

    def build_response(self,request):
        return run_recognition_pb2.ObjectRecognitionResponse()


class MockCameraOriginService(get_camera_origin_rpcz.CameraOriginService, BaseService):

    def build_response(self,request,reply):
        return get_camera_origin_pb2.CameraOriginResponse()


class MockCheckGraspReachabilityService(check_grasp_reachability_rpcz.CheckGraspReachabilityService, BaseService):

    def build_response(self,request,reply):
        return check_grasp_reachability_pb2.CheckGraspReachabilityResponse()


class MockExecuteGraspService(execute_grasp_rpcz.ExecuteGraspService, BaseService):

    def build_response(self,request,reply):
        return execute_grasp_pb2.ExecuteGraspResponse()
