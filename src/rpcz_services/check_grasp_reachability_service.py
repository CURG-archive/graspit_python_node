
from gen_proto import check_grasp_reachability_pb2
from gen_proto import check_grasp_reachability_rpcz

from base_service import BaseService

from moveit_trajectory_planner.srv import *

import graspit_msgs.msg
import rospy

class CheckGraspReachabilityService(check_grasp_reachability_rpcz.CheckGraspReachabilityService, BaseService):

    def __init__(self, ros_interface):
        super(CheckGraspReachabilityService, self).__init__(ros_interface)


    def build_response(self, request):
        #self.save_proto_msg_for_testing(request)
        response = check_grasp_reachability_pb2.CheckGraspReachabilityResponse()

        proxy_request = self.build_grasp_msg(request)

        check_reachability_ros_response = self.ros_interface.handle_check_reachability_request(proxy_request)

        response.graspId = request.grasp.graspId
        response.graspStatus = check_reachability_ros_response.isPossible

        return response



    def build_grasp_msg(self, request):
        grasp_msg = graspit_msgs.msg.Grasp()

        grasp_msg.object_name = request.grasp.object.name
        grasp_msg.epsilon_quality = request.grasp.epsilon_quality

        grasp_msg.pre_grasp_pose.position.x = request.grasp.pre_grasp_hand_state.hand_pose.position.x
        grasp_msg.pre_grasp_pose.position.y = request.grasp.pre_grasp_hand_state.hand_pose.position.y
        grasp_msg.pre_grasp_pose.position.z = request.grasp.pre_grasp_hand_state.hand_pose.position.z
        grasp_msg.pre_grasp_pose.orientation.x = request.grasp.pre_grasp_hand_state.hand_pose.orientation.x
        grasp_msg.pre_grasp_pose.orientation.y = request.grasp.pre_grasp_hand_state.hand_pose.orientation.y
        grasp_msg.pre_grasp_pose.orientation.z = request.grasp.pre_grasp_hand_state.hand_pose.orientation.z
        grasp_msg.pre_grasp_pose.orientation.w = request.grasp.pre_grasp_hand_state.hand_pose.orientation.w

        grasp_msg.final_grasp_pose.position.x = request.grasp.final_grasp_hand_state.hand_pose.position.x
        grasp_msg.final_grasp_pose.position.y = request.grasp.final_grasp_hand_state.hand_pose.position.y
        grasp_msg.final_grasp_pose.position.z = request.grasp.final_grasp_hand_state.hand_pose.position.z
        grasp_msg.final_grasp_pose.orientation.x = request.grasp.final_grasp_hand_state.hand_pose.orientation.x
        grasp_msg.final_grasp_pose.orientation.y = request.grasp.final_grasp_hand_state.hand_pose.orientation.y
        grasp_msg.final_grasp_pose.orientation.z = request.grasp.final_grasp_hand_state.hand_pose.orientation.z
        grasp_msg.final_grasp_pose.orientation.w = request.grasp.final_grasp_hand_state.hand_pose.orientation.w

        grasp_msg.pre_grasp_dof = request.grasp.pre_grasp_hand_state.hand_dof
        grasp_msg.final_grasp_dof = request.grasp.final_grasp_hand_state.hand_dof

        grasp_msg.secondary_qualities = [0.0]
        return grasp_msg

    def save_proto_msg_for_testing(self,proto_request):
        f = open("grasp_proto_" + str(proto_request.grasp.graspId) + str(".saved_proto"), "wb")
        f.write(proto_request.SerializeToString())
        f.close()