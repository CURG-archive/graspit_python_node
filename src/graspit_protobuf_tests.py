#!/bin/python
__author__ = 'jweisz'
import socket
import gen_proto
import gen_proto.GraspitMessage_pb2
import gen_proto.Renderable_pb2
import struct
import rospy
import graspit_protobuf_socket
import rospy



class MessageTester(object):
    def __init__(self):
        self.sock = self.get_test_socket()


    def send_empty_message(self):
        """
        :param sock: a graspit_protobuf_socket to send the message on
        :type sock: graspit_protobuf_socket.GraspitProtobufSocket
        :rtype:None
        """
        empty_msg = gen_proto.GraspitMessage_pb2.GraspitProtobufMessage()
        assert isinstance(self.sock, graspit_protobuf_socket.GraspitProtobufSocket)
        self.sock.send_proto(empty_msg)


    def get_test_socket(self):
        """
        Create a graspit socket using rosparams for the graspit host name and port, or use defaults
        :return: A GraspitProtobufSocket for sending messages to graspit
        :rtype: graspit_protobuf_socket.GraspitProtobufSocket
        """
        host = rospy.get_param("graspit_host_name", 'localhost')
        port = rospy.get_param("graspit_socket", 4766)
        assert isinstance(host, basestring)
        assert isinstance(port, int)
        rospy.loginfo("hostname:%s portnum:%i"%(host, port))
        sock = graspit_protobuf_socket.GraspitProtobufSocket(host=host, port=port)
        return sock

    def send_pointcloud_message(self):
        msg = gen_proto.GraspitMessage_pb2.GraspitProtobufMessage()
        points = msg.renderable.pointCloud.points
        points.add()
        points[-1].point.x = 0
        points[-1].point.y = 1
        points[-1].point.z = 2
        points[-1].color.red = .1
        points[-1].color.green = .2
        points[-1].color.blue = .3
        self.sock.send_proto(msg)

if __name__ == "__main__":
    rospy.init_node("graspit_protobuf_socket_tester")

    run_empty_test = rospy.get_param("run_empty_test", False)
    run_pointcloud_test = rospy.get_param("run_pointcloud_test", True)
    if run_empty_test:
        tester = MessageTester()
        tester.send_empty_message()

    if run_pointcloud_test:
        tester = MessageTester()
        tester.send_pointcloud_message()

