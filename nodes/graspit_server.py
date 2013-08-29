#!/usr/bin/env python
'''
@package middle-ware package to communicate with graspit and convert it to ros message
'''

import roslib
roslib.load_manifest( "graspit_python_node" )
import rospy
import socket, time
import graspit_msgs.msg
import geometry_msgs.msg
import tf, tf_conversions, tf.transformations
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs
import pdb
from graspit_commands import *
import std_msgs.msg
import tf_conversions.posemath as pm





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

        
def parse_unstructured_grasp_string(grasp_string):
    #clean the string from extraneous characters    
    gs = grasp_string.strip('{').rstrip('\n').rstrip('}').rstrip(',')
    grasp_list = gs.split(',')
    g = list()
    for grasp in grasp_list:
        #split into unstructured list
        gs_list = grasp.split(' ')
        #extract grasp fields
        quat_list = [float(x) for x in gs_list[0:4]]
        tran_list = [float(x) for x in gs_list[4:7]]
    
        dof_list = [float(x) for x in gs_list[7:11]]
        epsilon_quality = float(gs_list[11])
        volume_quality = float(gs_list[12])
        quality_list = []
        if len(gs_list) > 13:
            quality_list = [float(x) for x in gs_list[13:-1]]

    
        q = geometry_msgs.msg.Quaternion(w = quat_list[0], x = quat_list[1], y = quat_list[2], z = quat_list[3])
        
        p = geometry_msgs.msg.Point(x = tran_list[0], y = tran_list[1], z = tran_list[2])
        grasp_pose_msg = geometry_msgs.msg.Pose(orientation = q, position = p)
        g.append(graspit_msgs.msg.Grasp(epsilon_quality = epsilon_quality, pre_grasp_pose = grasp_pose_msg, final_grasp_pose = grasp_pose_msg, pre_grasp_dof = dof_list, final_grasp_dof = dof_list, secondary_qualities = [0.0]))
        
    return g
        
    
    

class GraspitExecutionListener( object ):
    def __init__(self, hostname, pointcloud_topic):
        self.socket = []
        self.graspit_commander = []        
        self.hostname = hostname
        while not self.try_reconnect():
            print "initial connection failed. Reconnecting \n"
        self.grasp_pub = rospy.Publisher('/graspit/grasps', graspit_msgs.msg.Grasp)
        self.analyze_grasp_pub = rospy.Publisher('/graspit/analyze_grasps', graspit_msgs.msg.Grasp)
        
        #self.target_subscriber = rospy.Subscriber('/graspit/target_name', std_msgs.msg.String, self.set_target)
        self.object_subscriber = rospy.Subscriber('/graspit/object_name', graspit_msgs.msg.ObjectInfo, self.add_object)
        self.clear_objects_subscriber = rospy.Subscriber('/graspit/remove_objects', std_msgs.msg.String, self.clear_objects)
        self.object_recognition_pub = rospy.Publisher('/graspit/refresh_models', std_msgs.msg.Empty)
        self.target_name_pub = rospy.Publisher('/graspit/target_name', std_msgs.msg.String)
        self.transform_listener = tf.TransformListener()
        self.error_error_subscriber = rospy.Subscriber("/graspit/status", graspit_msgs.msg.GraspStatus, self.send_error)
        self.target_name = ""
        self.target_transform = eye(4)
        self.pointcloud_topic = pointcloud_topic

    def try_reconnect(self):
        try:
            s = socket.socket()
            s.connect(self.hostname)
            s.settimeout(5)
            s.setblocking(False)
            self.socket = s
            self.graspit_commander = graspitManager(self.socket)
            return True
        except:
            return False

    def try_get_transform(self):
        try:
            self.transform_listener.waitForTransform("/camera_rgb_optical_frame", self.target_name, rospy.Time(0),rospy.Duration(.1))
        except:
            return False
        self.target_transform = tf_conversions.toMatrix(tf_conversions.fromTf(
                                self.transform_listener.lookupTransform(
                                "/camera_rgb_optical_frame", self.target_name, rospy.Time(0))))
        return True

    def try_get_world_transform(self):
        try:
            self.transform_listener.waitForTransform("/camera_rgb_optical_frame", "/world", rospy.Time(0),rospy.Duration(.1))
        except:
            return False
        self.target_transform = tf_conversions.toMatrix(tf_conversions.fromTf(
            self.transform_listener.lookupTransform(
                "/camera_rgb_optical_frame",'/world', rospy.Time(0))))
        return True
    
    def update_object_pointcloud(self):

        def get_point():
            pc = rospy.wait_for_message(self.pointcloud_topic, sensor_msgs.msg.PointCloud2,2)
            return pc
        print "updating object pointcloud "
        if self.try_get_transform() or self.try_get_world_transform():
            try:
                pc = get_point()
            except:
                return

            self.graspit_commander.send_pointlist_to_graspit(pc, 5, -1, np.linalg.inv(self.target_transform))
            print "got transform"
        else:
            print "Failed to get transform"
            
#        print "updating object pointcloud in graspit"
#        if self.try_get_transform():
#            pc = get_point()
#            point_list = self.graspit_commander.send_pointcloud_to_graspit(pc, 20)
#            self.graspit_commander.send_pointlist_to_graspit(point_list, -1, np.linalg.inv(self.target_transform))
#
#        else:
#            print "Failed to get transform"
#        return
        
        
        
        
    def run_object_recognition(self):
        self.object_recognition_pub.publish()

    def set_target(self, msg):
        self.target_name = msg.data
        self.graspit_commander.set_planner_target(self.target_name)
        if not self.try_get_transform():
            print "Unable to get transform"
        #else:
            #self.graspit_commander.set_camera_origin(np.linalg.inv(self.target_transform)[0:3,3])

    def add_object(self, msg):
        model_name=msg.object_name.split(' ')[0]
        object_name=msg.object_name.split(' ')[1]        
        self.graspit_commander.add_object(model_name, object_name, msg.object_pose)
        


    def send_error(self, msg):
        self.graspit_commander.send_grasp_failed(0, msg.grasp_status, msg.status_msg)

    def clear_objects(self, msg):
        self.graspit_commander.clear_objects(msg.data)

    def parse_grasp_string(self, grasp_string):
        """Parse string returned to graspit in to grasp message
        FIXME
        This and all of it's helper functions should move in to graspit_commands.py

        """
        grasp_msg = parse_unstructured_grasp_string(grasp_string)
        final_grasp_pose = self.graspit_commander.get_current_hand_pose_msg()
        grasp_msg[0].final_grasp_pose = final_grasp_pose
        return grasp_msg
        
    def try_read(self):        
        try:
            received_string = self.socket.recv(4096)
        except Exception as e:            
            if isinstance(e, socket.timeout):
                return [],[]
        
        if received_string == "":
            print "trying to reconnect"
            self.socket = []
            self.try_reconnect()
            return [],[]        
        
        for line in received_string.split('\n'):
            words = line.split(' ')
            if words[0] == "runObjectRecognition":
                self.run_object_recognition()
                #self.clear_objects(std_msgs.msg.String('blah'))
                #self.target_name = ""
                #self.target_name_pub.publish(self.target_name)
                #self.try_get_transform()
                return [], "runObjectRecogntion"

            if words[0] == 'setTarget' and len(words) > 1:
                self.target_name = words[1].rstrip('\n')
                self.target_name_pub.publish(self.target_name)
                if not self.try_get_transform():                
                    print "Unable to get transform"
                return [], 'setTarget'

            if words[0] == 'analyzeGrasp':
                grasp_line = ' '.join(words[1:])
                grasp_msg = self.parse_grasp_string(grasp_line)
                self.analyze_grasp_pub(grasp_msg)
                
            grasp_msg = []
            try:
                grasp_msg = self.parse_grasp_string(line)            
                self.grasp_pub.publish(grasp_msg[0])
            except Exception as e:                
                print "error parsing line: %s"%(line)

        return received_string, grasp_msg



    def update_table(self):
        try:
            trans = tf_conversions.toMatrix(tf_conversions.fromTf(self.transform_listener.lookupTransform('world','object', rospy.Time(0))))
            trans[0:3,3] *= 1000 
            trans[2,3] -=  70
            self.graspit_commander.set_body_trans('experiment_table', trans)

        except:
            pdb.post_mortem()
        


if __name__ == '__main__':
     try:
         rospy.init_node('graspit_python_server')
         send_cloud = rospy.get_param('send_cloud',True)
         graspit_url = rospy.get_param('graspit_url','picard.cs.columbia.edu')
         pointcloud_topic = rospy.get_param('graspit_pointcloud_topic', '/camera/depth_registered/points')
         print graspit_url
         g = GraspitExecutionListener((graspit_url,4765), pointcloud_topic)

#         g = GraspitExecutionListener(('tonga.cs.columbia.edu',4765))
         g.graspit_commander.get_graspit_objects()
         print "got objects \n"
#         table_ind = g.graspit_commander.object_ind('experiment_table')
#         if not table_ind:
#             g.graspit_commander.add_object('experiment_table')            
#             g.graspit_commander.add_obstacle('experiment_table')            
         loop = rospy.Rate(10)
         while not rospy.is_shutdown():
             s,grasp_msg = g.try_read()
             if send_cloud:
                 g.update_object_pointcloud()
             print grasp_msg
             loop.sleep()
     except rospy.ROSInterruptException: pass
    
            




                        
