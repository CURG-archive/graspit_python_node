#!/usr/bin/env python

import threading

import rpcz
from ros_services.recognition_service import RecognitionService
from ros_services.camera_origin_service import CameraOriginService
from ros_services.check_grasp_reachability_service import CheckGraspReachabilityService
from ros_services.execute_grasp_service import ExecuteGraspService

import roslib
roslib.load_manifest( "graspit_python_node" )
import rospy


class RPCZAppManager():
    def __init__(self, server_address, services):
        self.app = rpcz.Application()
        self.appThread = ServerThread(self.app, server_address, services)

    def start(self):
        self.appThread.start()

    def shutdown(self):
        print("shutting down server")
        self.app.terminate()
        self.appThread.join()


class ServerThread(threading.Thread):
    def __init__(self, app, server_address, services):
        threading.Thread.__init__(self)
        self.app = app
        server = rpcz.Server(self.app)
        self.server_address = server_address

        for service in services:
            server.register_service(service(), service.__name__)

        server.bind(server_address)

    def run(self):
        print
        "Serving requests at: " + self.server_address
        self.app.run()



def ros_graspit_run():
    server_address = "tcp://*:5561"

    services = [RecognitionService,
                CameraOriginService,
                CheckGraspReachabilityService,
                ExecuteGraspService]

    rpcz_app_manager = RPCZAppManager(server_address, services)
    try:
        rpcz_app_manager.start()

        rospy.init_node('graspit_python_server')
        loop = rospy.Rate(10)
        rospy.loginfo("server started sucessfully")
        while not rospy.is_shutdown():
            try:
                loop.sleep()
            except (KeyboardInterrupt, SystemExit):
                break

        rpcz_app_manager.shutdown()
    except:
        rpcz_app_manager.shutdown()


if __name__ == "__main__":
    rospy.loginfo("starting server")
    ros_graspit_run()
