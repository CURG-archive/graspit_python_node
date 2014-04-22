import graspit_msgs.msg
from gen_proto import grasp_pb2
import roslib.packages

def build_grasp_msg(rpcz_grasp_msg):
        ros_grasp_msg = graspit_msgs.msg.Grasp()

        ros_grasp_msg.object_name = rpcz_grasp_msg.object.name
        ros_grasp_msg.epsilon_quality = rpcz_grasp_msg.epsilon_quality

        ros_grasp_msg.pre_grasp_pose.position.x = rpcz_grasp_msg.pre_grasp_hand_state.hand_pose.position.x
        ros_grasp_msg.pre_grasp_pose.position.y = rpcz_grasp_msg.pre_grasp_hand_state.hand_pose.position.y
        ros_grasp_msg.pre_grasp_pose.position.z = rpcz_grasp_msg.pre_grasp_hand_state.hand_pose.position.z
        ros_grasp_msg.pre_grasp_pose.orientation.x = rpcz_grasp_msg.pre_grasp_hand_state.hand_pose.orientation.x
        ros_grasp_msg.pre_grasp_pose.orientation.y = rpcz_grasp_msg.pre_grasp_hand_state.hand_pose.orientation.y
        ros_grasp_msg.pre_grasp_pose.orientation.z = rpcz_grasp_msg.pre_grasp_hand_state.hand_pose.orientation.z
        ros_grasp_msg.pre_grasp_pose.orientation.w = rpcz_grasp_msg.pre_grasp_hand_state.hand_pose.orientation.w

        ros_grasp_msg.final_grasp_pose.position.x = rpcz_grasp_msg.final_grasp_hand_state.hand_pose.position.x
        ros_grasp_msg.final_grasp_pose.position.y = rpcz_grasp_msg.final_grasp_hand_state.hand_pose.position.y
        ros_grasp_msg.final_grasp_pose.position.z = rpcz_grasp_msg.final_grasp_hand_state.hand_pose.position.z
        ros_grasp_msg.final_grasp_pose.orientation.x = rpcz_grasp_msg.final_grasp_hand_state.hand_pose.orientation.x
        ros_grasp_msg.final_grasp_pose.orientation.y = rpcz_grasp_msg.final_grasp_hand_state.hand_pose.orientation.y
        ros_grasp_msg.final_grasp_pose.orientation.z = rpcz_grasp_msg.final_grasp_hand_state.hand_pose.orientation.z
        ros_grasp_msg.final_grasp_pose.orientation.w = rpcz_grasp_msg.final_grasp_hand_state.hand_pose.orientation.w

        ros_grasp_msg.pre_grasp_dof = rpcz_grasp_msg.pre_grasp_hand_state.hand_dof
        ros_grasp_msg.final_grasp_dof = rpcz_grasp_msg.final_grasp_hand_state.hand_dof

        ros_grasp_msg.secondary_qualities = [0.0]
        return ros_grasp_msg


def save_rpcz_request(rpcz_request, filename):
        pkg_path = roslib.packages.get_pkg_dir('mock_graspit') + '/'
        f = open(pkg_path + filename, "wb")
        f.write(rpcz_request.SerializeToString())
        f.close()