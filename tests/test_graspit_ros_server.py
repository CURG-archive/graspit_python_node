
import graspit_ros_server
from graspit_mock_services import *


def run_test_graspit_ros_server():
    server_address = "tcp://*:5561"

    services = [MockObjectRecognitionService,
                MockCameraOriginService,
                MockCheckGraspReachabilityService,
                MockExecuteGraspService]

    graspit_ros_server.run_graspit_ros_node(server_address, services)


if __name__ == "__main__":
    run_test_graspit_ros_server()