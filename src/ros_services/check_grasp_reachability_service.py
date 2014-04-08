
from gen_proto import check_grasp_reachability_pb2
from gen_proto import check_grasp_reachability_rpcz

from base_service import BaseService

from moveit_trajectory_planner.srv import *
import rospy


class CheckGraspReachabilityService(check_grasp_reachability_rpcz.CheckGraspReachabilityService, BaseService):

    def __init__(self):
        super(CheckGraspReachabilityService, self).__init__()

        rospy.wait_for_service('moveit_trajectory_planner/check_reachability')
        self.check_reachability_proxy = rospy.ServiceProxy('moveit_trajectory_planner/check_reachability', LocationInfo)

    def build_response(self, request):
        response = check_grasp_reachability_pb2.CheckGraspReachabilityResponse()

        proxy_request = self.convert_proto_to_ros_service_request(request)

        check_reachability_ros_response = self.check_reachability_proxy(proxy_request)

        response.graspId = request.grasp.graspId
        response.graspStatus = check_reachability_ros_response.isPossible

        return response

    def convert_proto_to_ros_service_request(self, request):

        check_reachability_request = LocationInfo.Request
        check_reachability_request.position.x = request.grasp.final_grasp_hand_state.hand_pose.position.x
        check_reachability_request.position.y = request.grasp.final_grasp_hand_state.hand_pose.position.y
        check_reachability_request.position.z = request.grasp.final_grasp_hand_state.hand_pose.position.z

        check_reachability_request.orientation.x = request.grasp.final_grasp_hand_state.hand_pose.orientation.x
        check_reachability_request.orientation.y = request.grasp.final_grasp_hand_state.hand_pose.orientation.y
        check_reachability_request.orientation.z = request.grasp.final_grasp_hand_state.hand_pose.orientation.z
        check_reachability_request.orientation.w = request.grasp.final_grasp_hand_state.hand_pose.orientation.w

        return check_reachability_request