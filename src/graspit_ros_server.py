#!/usr/bin/env python

import threading

import rpcz
from rpcz_services.object_recognition_service import ObjectRecognitionService
from rpcz_services.camera_origin_service import CameraOriginService
from rpcz_services.check_grasp_reachability_service import CheckGraspReachabilityService
from rpcz_services.execute_grasp_service import ExecuteGraspService

import roslib
import rospy

from ros_request_handlers.ros_interface import ROSInterface

roslib.load_manifest("graspit_python_node")


class ServerThread(threading.Thread):
    def __init__(self, server_address,services):
        threading.Thread.__init__(self)
        self.daemon = True

        self.server_address = server_address
        self.services = services

    def run(self):
        app = rpcz.Application()
        server = rpcz.Server(app)

        for service in self.services:
            print "registering service: " + str(service.__class__.__name__) + " as rpcz service: " + str(service.DESCRIPTOR.name)
            rospy.loginfo("registering service: " + str(service.__class__.__name__) + " as rpcz service: " + str(service.DESCRIPTOR.name))
            server.register_service(service, service.DESCRIPTOR.name)

        server.bind(self.server_address)
        rospy.loginfo("Serving requests on port " + str(self.server_address))
        app.run()


def run_graspit_ros_node(server_address, services):

    rospy.loginfo("launching graspit_ros_server!!!!!!!!!!!!!!")
    rpcz_server = ServerThread(server_address, services)
    rpcz_server.start()

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
    services = [ObjectRecognitionService(ros_interface),
                CameraOriginService(ros_interface),
                CheckGraspReachabilityService(ros_interface),
                ExecuteGraspService(ros_interface)]

    run_graspit_ros_node(server_address,services)
