
from gen_proto import execute_grasp_pb2
from gen_proto import execute_grasp_rpcz

from base_service import BaseService

import graspit_msgs.msg
import rospy


class ExecuteGraspService(execute_grasp_rpcz.ExecuteGraspService, BaseService):

    def __init__(self):
        super(ExecuteGraspService, self).__init__()
        self.grasp_pub = rospy.Publisher('/graspit/grasps', graspit_msgs.msg.Grasp)

    def build_response(self, request):
        grasp_msg = self.build_grasp_msg(request)
        self.grasp_pub.publish(grasp_msg)
        return execute_grasp_pb2.ExecuteGraspResponse()

    def build_grasp_msg(self, request):

        grasp_msg = graspit_msgs.msg.Grasp

        grasp_msg.object_name = request.object.name
        grasp_msg.epsilon_quality = request.epsilon_quality

        grasp_msg.pre_grasp_pose.position.x = request.pre_grasp_hand_state.hand_pose.position.x
        grasp_msg.pre_grasp_pose.position.y = request.pre_grasp_hand_state.hand_pose.position.y
        grasp_msg.pre_grasp_pose.position.z = request.pre_grasp_hand_state.hand_pose.position.z
        grasp_msg.pre_grasp_pose.orientation.x = request.pre_grasp_hand_state.hand_pose.orientation.x
        grasp_msg.pre_grasp_pose.orientation.y = request.pre_grasp_hand_state.hand_pose.orientation.y
        grasp_msg.pre_grasp_pose.orientation.z = request.pre_grasp_hand_state.hand_pose.orientation.z
        grasp_msg.pre_grasp_pose.orientation.w = request.pre_grasp_hand_state.hand_pose.orientation.w

        grasp_msg.final_grasp_pose.position.x = request.final_grasp_hand_state.hand_pose.position.x
        grasp_msg.final_grasp_pose.position.y = request.final_grasp_hand_state.hand_pose.position.y
        grasp_msg.final_grasp_pose.position.z = request.final_grasp_hand_state.hand_pose.position.z
        grasp_msg.final_grasp_pose.orientation.x = request.final_grasp_hand_state.hand_pose.orientation.x
        grasp_msg.final_grasp_pose.orientation.y = request.final_grasp_hand_state.hand_pose.orientation.y
        grasp_msg.final_grasp_pose.orientation.z = request.final_grasp_hand_state.hand_pose.orientation.z
        grasp_msg.final_grasp_pose.orientation.w = request.final_grasp_hand_state.hand_pose.orientation.w

        grasp_msg.pre_grasp_dof = request.pre_grasp_hand_state.hand_dof
        grasp_msg.final_grasp_dof = request.final_grasp_hand_state.hand_dof

        grasp_msg.secondary_qualities = [0.0]

        return grasp_msg