#!/usr/bin/env python

import threading

import rpcz
from ros_services.object_recognition_service import ObjectRecognitionService
from ros_services.camera_origin_service import CameraOriginService
from ros_services.check_grasp_reachability_service import CheckGraspReachabilityService
from ros_services.execute_grasp_service import ExecuteGraspService

import roslib
import rospy
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
            rospy.loginfo("registering service: " + str(service.__name__) + " as rpcz service: " + str(service.DESCRIPTOR.name))
            server.register_service(service(), service.DESCRIPTOR.name)

        server.bind(self.server_address)
        rospy.loginfo("Serving requests on port " + str(self.server_address))
        app.run()


def run_graspit_ros_node(server_address,services):

    rospy.init_node('graspit_python_server')

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
    server_address = "tcp://*:5561"

    services = [ObjectRecognitionService,
                CameraOriginService,
                CheckGraspReachabilityService,
                ExecuteGraspService]

    run_graspit_ros_node(server_address,services)
