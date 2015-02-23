#!/usr/bin/env python

import threading

import rpcz
from rpcz_services.object_recognition_service import ObjectRecognitionService
from rpcz_services.camera_origin_service import CameraOriginService
from rpcz_services.check_grasp_reachability_service import CheckGraspReachabilityService
from rpcz_services.execute_grasp_service import ExecuteGraspService
from rpcz_services.get_option_selection import OptionSelectionService

import roslib
import rospy

from ros_request_handlers.ros_interface import ROSInterface

roslib.load_manifest("graspit_python_node")
from bondpy import bondpy

class ServerThread(threading.Thread):
    def __init__(self, server_address,services, service_names):
        threading.Thread.__init__(self)
        self.daemon = True
        self.service_names = service_names
        self.server_address = server_address
        self.services = services

    def run(self):
        app = rpcz.Application()
        server = rpcz.Server(app)

        for service, service_name in zip(self.services, self.service_names):
            print "registering service: " + str(service.__class__.__name__) + " as rpcz service: " + str(service.DESCRIPTOR.name)
            rospy.loginfo("registering service: " + str(service.__class__.__name__) + " as rpcz service: " + str(service.DESCRIPTOR.name))
            server.register_service(service, service_name)

        server.bind(self.server_address)
        rospy.loginfo("Serving requests on port " + str(self.server_address))
        app.run()


def run_graspit_ros_node(server_address, services, service_names):
    rospy.loginfo("launching graspit_ros_server!!!!!!!!!!!!!!")
    rpcz_server = ServerThread(server_address, services, service_names)
    rpcz_server.start()
    bond = bondpy.Bond("GRASPIT_ROS_SERVER", "test123")
    bond.start()
    loop = rospy.Rate(10)
    while (not rospy.is_shutdown()) and rpcz_server.is_alive():
        try:
            loop.sleep()
        except (KeyboardInterrupt, SystemExit):
            break
    rpcz_server.join(1)


if __name__ == "__main__":
    rospy.init_node('graspit_ros_server')
    server_address = "tcp://*:5561"
    ros_interface = ROSInterface()
    retrieval_service = ObjectRecognitionService(ros_interface)
#    retrieval_service.DESCRIPTOR.name = "RetrieveObjectsService"
    retrieval_service.isRetrieval = True

    services = [ObjectRecognitionService(ros_interface),
                OptionSelectionService(ros_interface),
                retrieval_service,
                CameraOriginService(ros_interface),
                CheckGraspReachabilityService(ros_interface),
                ExecuteGraspService(ros_interface)]
    service_names = [s.DESCRIPTOR.name for s in services]
    service_names[1] = "RetrieveObjectsService"
    run_graspit_ros_node(server_address,services, service_names)
