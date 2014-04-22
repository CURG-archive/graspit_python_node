
from gen_proto import check_grasp_reachability_pb2
from gen_proto import check_grasp_reachability_rpcz

from base_service import BaseService

from moveit_trajectory_planner.srv import *

import graspit_msgs.msg
import rospy
import utils

class CheckGraspReachabilityService(check_grasp_reachability_rpcz.CheckGraspReachabilityService, BaseService):

    def __init__(self, ros_interface):
        super(CheckGraspReachabilityService, self).__init__(ros_interface)


    def build_response(self, request):
        #utils.save_rpcz_request(request, "check_grasp_request" + str(request.grasp.graspId) + str(".saved_proto"))
        response = check_grasp_reachability_pb2.CheckGraspReachabilityResponse()

        proxy_request = utils.build_grasp_msg(request.grasp)

        check_reachability_ros_response = self.ros_interface.handle_check_reachability_request(proxy_request)

        response.graspId = request.grasp.graspId
        response.graspStatus = check_reachability_ros_response.isPossible

        return response

