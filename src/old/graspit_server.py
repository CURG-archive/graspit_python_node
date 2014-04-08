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
        object_name = gs_list[0]
        quat_list = [float(x) for x in gs_list[1:5]]
        tran_list = [float(x) for x in gs_list[5:8]]
    
        dof_list = [float(x) for x in gs_list[8:12]]
        epsilon_quality = float(gs_list[12])
        volume_quality = float(gs_list[13])
        quality_list = []
        if len(gs_list) > 14:
            quality_list = [float(x) for x in gs_list[14:] if x is not '']

    
        q = geometry_msgs.msg.Quaternion(w = quat_list[0], x = quat_list[1], y = quat_list[2], z = quat_list[3])
        
        p = geometry_msgs.msg.Point(x = tran_list[0], y = tran_list[1], z = tran_list[2])
        grasp_pose_msg = geometry_msgs.msg.Pose(orientation = q, position = p)
        g.append(graspit_msgs.msg.Grasp(object_name = object_name, epsilon_quality = epsilon_quality, pre_grasp_pose = grasp_pose_msg, final_grasp_pose = grasp_pose_msg, pre_grasp_dof = dof_list, final_grasp_dof = dof_list, secondary_qualities = [0.0]))
        
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
        self.analyze_approach_pub = rospy.Publisher('/graspit/analyze_demo_pose', graspit_msgs.msg.Grasp)
        
        #self.target_subscriber = rospy.Subscriber('/graspit/target_name', std_msgs.msg.String, self.set_target)
        self.object_subscriber = rospy.Subscriber('/graspit/scene_info', graspit_msgs.msg.SceneInfo, self.add_scene)
        self.clear_objects_subscriber = rospy.Subscriber('/graspit/remove_objects', std_msgs.msg.String, self.clear_objects)
        self.analysis_results_subsciber = rospy.Subscriber("/graspit/analyze_grasps_results", graspit_msgs.msg.GraspStatus, self.grasp_test_results)
        self.pose_results_subsciber = rospy.Subscriber("/graspit/analyze_demo_pose_results", std_msgs.msg.Float32, self.pose_test_results)
        
        self.object_recognition_pub = rospy.Publisher('/graspit/refresh_models', std_msgs.msg.Empty)
        self.transform_listener = tf.TransformListener()
        self.error_error_subscriber = rospy.Subscriber("/graspit/status", graspit_msgs.msg.GraspStatus, self.send_error)
        self.target_name = ""
        self.target_transform = eye(4)
        self.world_transform = eye(4)
        self.pointcloud_topic = pointcloud_topic
        self.remainder_string = ""
    def try_reconnect(self):
        try:
            s = socket.socket()
            s.connect(self.hostname)
            s.settimeout(5)
            s.setblocking(False)
            self.socket = s
            self.graspit_commander = graspitManager(self.socket)
            return bool(self.graspit_commander.connect_to_planner())
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
        self.world_transform = tf_conversions.toMatrix(tf_conversions.fromTf(
            self.transform_listener.lookupTransform(
                "/camera_rgb_optical_frame",'/world', rospy.Time(0))))
        return True
    
    def update_object_pointcloud(self):

        def get_point():
            pc = rospy.wait_for_message(self.pointcloud_topic, sensor_msgs.msg.PointCloud2,2)
            return pc
        print "updating object pointcloud "
        if self.try_get_world_transform():
            try:
                pc = get_point()
            except:
                return ''

            self.graspit_commander.send_pointlist_to_graspit(pc, 5, -1, np.linalg.inv(self.world_transform))
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
        if self.try_get_world_transform():
            self.graspit_commander.set_camera_origin(np.linalg.inv(self.world_transform)[0:3,3])
        
        self.object_recognition_pub.publish()


    def add_scene(self, scene_msg):
        self.graspit_commander.clear_objects("ALL")
        for obj_msg in scene_msg.objects:            
            self.graspit_commander.add_object(obj_msg.model_name, obj_msg.object_name, obj_msg.object_pose)
        
    def grasp_test_results(self, msg):
        grasp_status = msg.grasp_status
        if(msg.grasp_status == graspit_msgs.msg.GraspStatus.SUCCESS):
            grasp_status = -1
        
        
        self.graspit_commander.set_grasp_attribute(msg.grasp_identifier, 'testResult', -grasp_status)

    def pose_test_results(self, msg):
        if msg.data == 0:
            redness = 1
        else:
            redness = 0
            
        self.graspit_commander.set_robot_color(0,[redness, 0, 0])


    def send_error(self, msg):
        self.graspit_commander.send_grasp_failed(0, msg.grasp_status, msg.status_msg)

    def clear_objects(self, msg):
        self.graspit_commander.clear_objects(msg.data)

    def parse_grasp_string(self, grasp_string, refine = True):
        """Parse string returned to graspit in to grasp message
        FIXME
        This and all of it's helper functions should move in to graspit_commands.py

        """
        grasp_msg = parse_unstructured_grasp_string(grasp_string)
        if refine:
            final_grasp_pose = self.graspit_commander.get_current_hand_pose_msg()            
            grasp_msg[0].final_grasp_pose = final_grasp_pose            
        
        return grasp_msg

    def parseCmdString(self, received_string):        
        def parseCmdLine(line):
            result = []
            try:
                words = line.split(' ')
                if words[0] == "runObjectRecognition":
                    self.run_object_recognition()
                    

                elif words[0] == 'analyzeGrasp':
                    grasp_line = ' '.join(words[2:])                    
                    grasp_msg = self.parse_grasp_string(grasp_line, False)[0]
                    grasp_msg.secondary_qualities[0] = float(words[1])
                    self.analyze_grasp_pub.publish(grasp_msg)
                    result = grasp_msg

                elif words[0] == 'analyzeApproachDirection':
                    grasp_line = ' '.join(words[1:])                    
                    grasp_msg = self.parse_grasp_string(grasp_line, False)[0]                    
                    self.analyze_approach_pub.publish(grasp_msg)
                    result = grasp_msg

                elif words[0] == "doGrasp":
                    grasp_msg = self.parse_grasp_string(' '.join(words[1:]), False)[0]
                    self.grasp_pub.publish(grasp_msg)
                    result = grasp_msg
                    
                else:
                    print "unknown line %s"%(line)
                return [words[0], result]
            except Exception as e:                
                print "error parsing line: %s"%(line)
                print e
                self.remainder_string = line
                return [],[]
        received_string = self.remainder_string + received_string
        self.remainder_string = ""
        return [parseCmdLine(line) for line in received_string.split('\n')]
        
            
        
    def try_read(self):
        
        try:
            received_string = self.socket.recv(4096)
        except Exception as e:
            self.socket = []
            self.try_reconnect()
            return [],[]
        
        
        if received_string == "":
            print "trying to reconnect"
            self.socket = []
            self.try_reconnect()
            return [],[]        
        results = self.parseCmdString(received_string)
        print results
        return 



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
         

         loop = rospy.Rate(10)
         while not rospy.is_shutdown():
             g.try_read()
             if send_cloud:
                 try:
                     g.update_object_pointcloud()
                 except Exception as e:
                     print e

             loop.sleep()
     except rospy.ROSInterruptException: pass
    
