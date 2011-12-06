import numpy as np
import roslib
roslib.load_manifest( "graspit_python_node" )
import tf
from tf_conversions import *
import re

unsigned_num_regex = "[0-9]*"
signed_num_regex = "[+-]?" + unsigned_num_regex
signed_exponent_regex = "[eE]?" + signed_num_regex
float_regex = R'[\+\-]?\d+[.\d*]?[+-]?[eE]?\d*'


class graspitManager():
    class graspit_object( object ):
        def __init__(self, object_name, object_transform):
            self.object_transform = object_transform
            self.object_name = object_name

    def __init__(self, socket):
        self.socket = socket
        self.socket.settimeout(3)
        self.graspable_body_list = []
        self.obstacle_list = []
        self.body_list = []
        self.robot_list = []

    def get_list_ind(self, object_name, object_list):
        return [x for x in range(len(object_list)) if object_list[x].object_name == object_name]

    def object_ind(self, object_name):
        return self.get_list_ind(object_name, self.body_list)

    def robot_ind(self, robot_name):
        return self.get_list_ind(robot_name, self.robot_list)

    def obstacle_ind(self, obstacle_name):
        return self.get_list_ind(obstacle_name, self.obstacle_list)

    def get_list_by_body(self, command_str):
        self.socket.send(command_str)
        return_str = self.socket.recv(4096)
        return_str_list = return_str.split('\n')        
        num_bodies = int(return_str_list[0])
        new_list = list()
        for i in range(1,num_bodies+1):
            new_list.append(return_str_list[i])
        return new_list

    def get_body_list(self):
        command_str = "getBodyName ALL \n"        
        body_list = self.get_list_by_body(command_str)
        return body_list

    def process_transform_string(self, tran_str):
        r = re.findall(float_regex, tran_str)
        return r

    def get_body_transforms(self):
        command_str = "sendBodyTransf ALL \n"
        transform_string_list = self.get_list_by_body(command_str)
        transform_matrix_list = list()
        for transform_string in transform_string_list:
            transform_list = [float(x) for x in self.process_transform_string(transform_string)]
            
            p = Pose(position = Point(transform_list[0], transform_list[1], transform_list[2]),
                     orientation = Quaternion(w = transform_list[3], x = transform_list[4],
                                  y = transform_list[5], z = transform_list[6]))
            transform_matrix_list.append(toMatrix(fromMsg(p)))
        return transform_matrix_list

    def get_graspit_objects(self):
        body_list = self.get_body_list()
        transform_list = self.get_body_transforms()
        self.body_list = [self.graspit_object(n,t) for n,t in zip(body_list, transform_list)]

    
    def add_obstacle(self, obstacle_name):
        command_str = "addObstacle " + obstacle_name + ".xml \n"
        self.socket.send(command_str)
        return self.socket.recv(100)

    def add_object(self, obstacle_name):
        command_str = "addObject " + obstacle_name + ".xml \n"
        self.socket.send(command_str)
        return self.socket.recv(100)

    def pose_msg_to_tran_string(self, pose_msg):
        return( "%f %f %f %f %f %f %f"%(pose_msg.position.x,
                                       pose_msg.position.y,
                                       pose_msg.position.z,
                                       pose_msg.orientation.w,
                                       pose_msg.orientation.x,
                                       pose_msg.orientation.y,
                                       pose_msg.orientation.z)
                )

    def set_bodyind_trans_from_pose_msg(self, object_ind, pose_msg):
        command_str = "setBodyTransf 1 %i "%(object_ind[0]) + self.pose_msg_to_tran_string(pose_msg) + '\n'
        self.socket.send(command_str)
        return self.socket.recv(100)


    def set_body_trans_from_pose_msg(self, object_name, pose_msg):
        return self.set_bodyind_trans_from_pose_msg(self.object_ind(object_name), pose_msg)

    def set_body_trans(self, object_name, trans_mat):
        pose_msg = toMsg(fromMatrix(trans_mat))
        return(self.set_body_trans_from_pose_msg(object_name, pose_msg))


    def set_relative_transform(self, anchor_body_name, rel_body_name, rel_trans):
        self.get_graspit_objects()
        anchor_body_trans = [x.object_transform for x in self.body_list if x.object_name == anchor_body_name][0]
        desired_trans = np.dot(anchor_body_trans, rel_trans)
        return self.set_body_trans(rel_body_name, rel_trans)
        

    def connect_world_planner(self):
        self.socket.send('connectToWorldPlanner \n')
        
    def send_grasp_unreachable(self, grasp_index):
        self.socket.send('signalGraspUnreachable %i \n'%(grasp_index) )
                         
        
    
               
