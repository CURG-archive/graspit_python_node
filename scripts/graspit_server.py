"""@package
middle-ware package to communicate with graspit and convert it to ros message
"""
import roslib; roslib.load_manifest( "graspit_python_node" )
import rospy
import asyncore
import socket, time
import graspit_msgs.msg
import geometry_msgs.msg
import tf, tf_conversions, tf.transformations
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs

#def main():
#   pass




#if __name__ == "__main__":
#    main()

test_string = '{0.588131 0.742387 -0.160745 -0.277714 -109.514 48.7265 10.208 0 1.58644 0.690816 0.783941 -5.38901e+305 -5.38901e+305,}\n'

class graspit_barrett_params(object):
    def __init__(self, grasp_pose_list, grasp_joints_list, quality_scores):
        self.spread_angle = grasp_joints_list[-1]
        self.finger_1 = grasp_joints_list[0]
        self.finger_2 = grasp_joints_list[1]
        self.finger_3 = grasp_joints_list[2]
        self.joint_angles = [self.spread_angle, self.finger_1, self.finger_2, self.finger_3]
        self.grasp_pose = geometry_msgs.msg.Pose( grasp_pose_list[:3] , tf_conversions.Quaternion(w = grasp_pose_list[3], x = grasp_pose_list[4], y = grasp_pose_list[5], z = grasp_pose_list[6]))
        self.quality_scores = quality_scores
        
        
class graspit_barrett_grasp(object):
    def __init__(self, pre_grasp_grasp, final_grasp_grasp):
        self.pre_grasp = pre_grasp_grasp
        self.final_grasp = final_grasp_grasp

    def get_quality(self):
        return self.final_grasp.quality_scores

    def get_pregrasp_qualities(self):
        return self.pre_grasp.quality_scores

    def to_grasp_msg(self):
        return graspit_msgs.msg.Grasp(
                  pre_grasp_pose = self.pre_grasp.pose,
                  final_grasp_pose = self.final_grasp.pose,
                  pre_grasp_dof = self.pre_grasp.joint_angles,
                  final_grasp_dof = self.final_grasp.joint_angles,
                  grasp_source = 0,
                  epsilon_quality = self.final_grasp.quality_scores[0],
                  volume_quality = self.final_grasp.quality_scores[1],
                  secondary_qualities = self.final_grasp.quality_scores[2:]
            )
    
                                      
            
                                      

def parse_db_grasp_string(grasp_string):
    unstructured_grasp_list = [x.lstrip('[').split(',') for x in grasp_string.rstrip('{').lstrip('}').split(']')]
    def grasp_list_to_barrett_grasp(grasp_item_list):
        return graspit_barrett_params(grasp_item_list[:4], grasp_item_list[4:8], grasp_item_list[8:])

    structured_grasp_list = [grasp_list_to_barrett_grasp(x) for x in unstructured_grasp_list]
    full_structured_grasp_list = [graspit_barret_grasp(structured_grasp_list[ind], structured_grasp_list[ind+1]) for x in range(0,len(pregrasp_grasp_list),2)]  
    return full_structured_grasp_list

def parse_graspstate_grasp_string(grasp_string):
    grasp_string = grasp_string.strip('\n')
    if grasp_string[0] != '{' or grasp_string[-1] != '}':
        raise Exception('bad_grasp_string','improper terminator')
    cleaned_string = grasp_string.strip('{').strip('}')
    grasp_list = cleaned_string.split(',')

    def parse_grasp_list(gs):
        split_str = transform_string.split(')')
        starting_str = split_str[0]
        
    
    
class GraspitExecutionListener(asyncore.dispatcher_with_send):
    def handle_connect(self, address):
        print 'connected'

    def handle_close(self):
        self.close()       

    def handle_read(self):
        return parse_grasp_string(self.recv(4096))
    
            


class GraspitCommunicator(asyncore.dispatcher):
    """
    An object that encapsulates the communication with GraspIt!

    
    @param hostname - URL or ip to computer running GraspIt!
    @param port_number - The port number that the GraspIt! server is listening on.
    """
    def __init__(self, hostname = 'tonga.cs.columbia.edu',port_number = 4567):
        asyncore.dispatcher.__init__(self)
        self.port = port
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((hostname, port))
        self.listen(5)

    
    def handle_connect(self, address):
        pass
    
    def handle_accept(self):
        sock, addr = self.accept()
        h = GraspExecutionListner(sock)



    


                        
