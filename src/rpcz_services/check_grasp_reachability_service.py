
from gen_proto import check_grasp_reachability_pb2
from gen_proto import check_grasp_reachability_rpcz

from base_service import BaseService
import rospy
import utils
import ipdb

class CheckGraspReachabilityService(check_grasp_reachability_rpcz.CheckGraspReachabilityService, BaseService):

    def __init__(self, ros_interface):
        super(CheckGraspReachabilityService, self).__init__(ros_interface)

    def build_response(self, request):
        try:

            response = check_grasp_reachability_pb2.CheckGraspReachabilityResponse()

            proxy_request = utils.build_grasp_msg(request.grasp)

            check_reachability_ros_response = self.ros_interface.handle_check_reachability_request(proxy_request)

            response.graspId = request.grasp.graspId
            response.graspStatus = check_reachability_ros_response.isPossible

            # if check_reachability_ros_response.isPossible:
            #     utils.save_rpcz(request, "valid_check_grasp_request" + str(request.grasp.graspId) + str(".saved_proto"))
            # else:
            #     utils.save_rpcz(request, "invalid_check_grasp_request" + str(request.grasp.graspId) + str(".saved_proto"))
        except Exception as e:
            rospy.logerr("Grasp reachability check error: %s"%e )
            ipdb.settrace()
        return response

