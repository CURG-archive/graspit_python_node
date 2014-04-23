
import graspit_ros_server
from rpcz_services.mock_rpcz_services import *

import rospy

def run_test_graspit_ros_server():
    rospy.init_node('graspit_ros_server')
    server_address = "tcp://*:5561"

    services = [MockObjectRecognitionService(None),
                MockCameraOriginService(None),
                MockCheckGraspReachabilityService(None),
                MockExecuteGraspService(None)]

    graspit_ros_server.run_graspit_ros_node(server_address, services)


if __name__ == "__main__":
    run_test_graspit_ros_server()