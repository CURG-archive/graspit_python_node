import numpy as np
import roslib
roslib.load_manifest( "graspit_python_node" )
import tf
from tf_conversions import *
import re
import socket
import sensor_msgs.msg
import point_cloud2
import collections
import struct

unsigned_num_regex = "[0-9]*"
signed_num_regex = "[+-]?" + unsigned_num_regex
signed_exponent_regex = "[eE]?" + signed_num_regex
float_regex = R'[\+\-]?\d+[.\d*]?[+-]?[eE]?\d*'


class graspitManager():
    class graspit_object( object ):
        def __init__(self, object_name, object_transform):
            self.object_transform = object_transform
            self.object_name = object_name

    def __init__(self, sock):
        self.socket = sock
        self.socket.settimeout(5)
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
        return_str = ''
        while True:
            try:
                return_str += self.socket.recv(50000)
            except:
                break        
        return_str_list = return_str.split('\n')        
        num_bodies = int(return_str_list[0])
        new_list = list()
        for i in range(1, num_bodies+1):
            if return_str_list[i] =='':
                continue
            new_list.append(return_str_list[i])
        return new_list, '\n'.join(return_str_list[num_bodies+1:])

    def get_body_list(self):
        command_str = "getBodyName ALL \n"        
        body_list, remainder_string = self.get_list_by_body(command_str)
        return body_list, remainder_string

    def process_transform_string(self, tran_str):
        r = re.findall(float_regex, tran_str)
        return r

    def transform_string_to_transform(self, transform_string):
        transform_list = [float(x) for x in self.process_transform_string(transform_string)]
            
        p = Pose(position = Point(transform_list[4], transform_list[5], transform_list[6]),
                 orientation = Quaternion(w = transform_list[0], x = transform_list[1],
                y = transform_list[2], z = transform_list[3]))
        return toMatrix(fromMsg(p))
    


    def get_body_transforms(self):
        command_str = "sendBodyTransf ALL \n"

        transform_string_list, remainder_string = self.get_list_by_body(command_str)
        transform_matrix_list = list()
        remainder_string = ""
        for transform_string in transform_string_list:
            transform_list = [float(x) for x in self.process_transform_string(transform_string)]
            
            p = Pose(position = Point(transform_list[0], transform_list[1], transform_list[2]),
                     orientation = Quaternion(w = transform_list[3], x = transform_list[4],
                                              y = transform_list[5], z = transform_list[6]))
            transform_matrix_list.append(toMatrix(fromMsg(p)))
                
        return transform_matrix_list, remainder_string

    def get_graspit_objects(self):
        body_list, remainder_string = self.get_body_list()
        transform_list, remainder_string2 = self.get_body_transforms()
        self.body_list = [self.graspit_object(n,t) for n,t in zip(body_list, transform_list)]
        return remainder_string + remainder_string2
    
    def add_obstacle(self, obstacle_name):
        command_str = "addObstacle " + obstacle_name + ".xml \n"
        self.socket.send(command_str)
        succeeded = self.socket.recv(100)
        self.get_graspit_objects()
        return succeeded

    def add_object(self, model_name, object_name = "", pose_msg = Pose()):
        if object_name == "":
            object_name = model_name
        pose_str = self.pose_msg_to_tran_string(pose_msg)
        command_str = "addObject " + model_name + ".xml " + object_name + " " + pose_str + " \n"
        self.socket.send(command_str)
        try:
            result = self.socket.recv(10)
        except:
            return False
        # got_objects = False
        # while not got_objects:
        #     try:
        #         self.get_graspit_objects()
        #         got_objects = True
        #         print "got objects in add object"
        #     except:
        #         print "failed to get objects"
                            
        return result

    def set_object_name(self, object_ind, object_name):
        command_str = "setBodyName %d % s\n"%(object_ind, object_name)
        self.socket.send(command_str)
    

    def set_planner_target(self, object_name):
        command_str = "setPlannerTarget " + object_name + ".xml \n"
        self.socket.send(command_str)        
        try:
            self.socket.recv(100)
        except:
            pass
        return True

    def clear_objects(self, object_string):
        command_str = "clearGraspableBodies %s \n"%(object_string)
        self.socket.send(command_str)        
        return True

    def pose_msg_to_tran_string(self, pose_msg):
        return( "%f %f %f %f %f %f %f"%(pose_msg.position.x*1000,
                                       pose_msg.position.y*1000,
                                       pose_msg.position.z*1000,
                                       pose_msg.orientation.w,
                                       pose_msg.orientation.x,
                                       pose_msg.orientation.y,
                                       pose_msg.orientation.z)
                )


    def set_bodyind_trans_from_pose_msg(self, object_ind, pose_msg):
        if object_ind == []:
            print "set_body_ind_trans_from_pose_msg: no object ind"
            return False
        
        command_str = "setBodyTransf 1 %i "%(object_ind[0]) + self.pose_msg_to_tran_string(pose_msg) + '\n'
        self.socket.send(command_str)
        return self.socket.recv(100)


    def set_body_trans_from_pose_msg(self, object_name, pose_msg):        
        result = self.set_bodyind_trans_from_pose_msg(self.object_ind(object_name), pose_msg)
        if not result:
            print "failed to add object_name %s"%(object_name)
        return result

    def set_body_trans(self, object_name, trans_mat):
        pose_msg = toMsg(fromMatrix(trans_mat))
        return(self.set_body_trans_from_pose_msg(object_name, pose_msg))


    def set_relative_transform(self, anchor_body_name, rel_body_name, rel_trans):
        self.get_graspit_objects()
        anchor_body_trans = [x.object_transform for x in self.body_list if x.object_name == anchor_body_name][0]
        desired_trans = np.dot(anchor_body_trans, rel_trans)
        return self.set_body_trans(rel_body_name, rel_trans)
        

    def send_grasp_failed(self, grasp_index, error_num, error_string):
        self.socket.send('signalGraspUnreachable %i %i %s\n'%(grasp_index, error_num, error_string) )
                         
    def get_current_hand_tran(self):
        self.socket.send('getCurrentHandTran \n')
        transform_string = self.socket.recv(300)
        transf = self.transform_string_to_transform(transform_string)
        return transf
               
    def get_current_hand_pose_msg(self):
        tran = self.get_current_hand_tran()
        return toMsg(fromMatrix(tran))
    

    def send_pointcloud_to_graspit(self, pointcloud_msg, downsample_factor = 1):
        uvs = [(u,v) for u in range(0, pointcloud_msg.height, downsample_factor)
               for v in range(0, pointcloud_msg.width,  downsample_factor)]
        point_generator = point_cloud2.read_points(pointcloud_msg,None, True, uvs)
        point_list = [point for point in point_generator]
        return point_list
        
    def send_pointlist_to_graspit(self, pointcloud_msg, downsample_factor = 1, num_points = -1 , tran = np.eye(4)):
        uvs = []
        if not downsample_factor == 1:
            uvs = [(u,v) for u in range(0, pointcloud_msg.height, downsample_factor)
                   for v in range(0, pointcloud_msg.width,  downsample_factor)]
        point_generator = point_cloud2.read_points(pointcloud_msg,None, True, uvs)
        point_list = [point for point in point_generator]
        color_list = [struct.unpack('BBBx', struct.pack('f',point[3])) for point in point_list]
        total_str = ""
        point_list2 = point_list[:num_points]
        for point_ind, point in enumerate(point_list2):
            color = color_list[point_ind]
            point_vec = tran*np.mat([point[0],point[1],point[2], 1]).transpose()
            point_str_list = " %f %f %f %i %i %i"%(point_vec[0], point_vec[1], point_vec[2],
                                                  color[2], color[1], color[0])
            total_str += point_str_list
        command_str = "addPointCloud %i%s \n"%(len(point_list2), total_str)
        self.socket.sendall(command_str)
        #success = bool(self.socket.recv(100))
        return command_str

    def set_camera_origin(self, origin_point):
        command_str = "setCameraOrigin %f %f %f \n"%(origin_point[0],
                                    origin_point[1], origin_point[2])
        self.socket.send(command_str)
        return command_str
            
    def remove_bodies(self, body_ind_list):
        body_str = None

        try:
            body_str = ' '.join([str(a) for a in body_ind_list])
        except:
            raise Exception('remove_bodies: body_ind of bad type')
        
        command_str = "removeBodies " + body_str + '\n'
        self.socket.send(command_str)
        return command_str
    
            
    def remove_bodies_by_name(self, body_names):
        body_name_list = None
        if isinstance(body_names, str):
            body_names_list = body_names.split(' ')
        elif isinstance(body_names, collections.Iterable):
            body_name_list = body_names
        else:
            raise Exception('remove_bodies_by_name: body_names of bad type')

        #translate body names to body inds
        body_ind_list = []
        [body_ind_list.extend(self.object_ind(object_name)) for object_name in body_name_list]
        return self.remove_bodies(body_ind_list)
        

    def set_grasp_attribute(self, grasp_id, attribute, value):
        """
        @param attribute - The string describing the name of the attribute.
        @param value - The value to set the attribute to.
        """
        command_str = "setGraspAttribute %f %s %f \n"%(grasp_id, attribute, value)
        self.socket.send(command_str)
        return command_str
            

