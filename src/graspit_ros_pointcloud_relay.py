#!/usr/bin/env python

import graspit_protobuf_socket
import gen_proto.GraspitMessage_pb2
import gen_proto.Renderable_pb2
import sensor_msgs.point_cloud2
import sensor_msgs.msg
import rospy
import struct
import tf
import tf_conversions
import numpy
from roslib import message
import ipdb

class GraspitProtobufSocketNode(object):
    def __init__ (self, point_cloud_topic="/camera/depth_registered/points", graspit_socket = [], downsample_factor = 2, skip_msgs = 0,
                  graspit_frame = 'world'):
        """
        :type point_cloud_topic: str
        :type graspit_socket: graspit_protobuf_socket.GraspitProtobufSocket
        """
        self.subscriber = rospy.Subscriber(point_cloud_topic, sensor_msgs.msg.PointCloud2, self.relay_message)
        if not graspit_socket:
            graspit_socket = graspit_protobuf_socket.GraspitProtobufSocket()
        self.graspit_socket = graspit_socket
        self.uvs = []
        self.last_message_size = (0,0)
        self.skip_msgs = 1
        self.downsample_factor = downsample_factor
        self.graspit_frame = graspit_frame
        self.listener = tf.TransformListener()
        self.listener.setUsingDedicatedThread(True)
        if downsample_factor:
            self.skip_msgs *= downsample_factor
        rospy.loginfo(self.__class__.__name__ + " is inited")

    def relay_message(self, pointcloud_msg):
        """
        :type pointcloud_msg :sensor_msgs.msg.PointCloud2
        """


        pointcloud_transform = tf_conversions.toMatrix(tf_conversions.fromTf(self.listener.lookupTransform(self.graspit_frame, pointcloud_msg.header.frame_id, rospy.Time(0))))

        rospy.loginfo(self.__class__.__name__ + " is relaying message")
        if pointcloud_msg.header.seq % self.skip_msgs:
            return
        if self.downsample_factor:
            self.create_uvs(pointcloud_msg)
        #points = sensor_msgs.point_cloud2.read_points(pointcloud_msg, None, True, self.uvs)
        points = sensor_msgs.point_cloud2.read_points(pointcloud_msg)

        gm = gen_proto.GraspitMessage_pb2.GraspitProtobufMessage()

        #renderable = gen_proto.Renderable_pb2.Renderable()
        #pointcloud = gen_proto.Renderable_pb2.Renderable.PointCloudXYZRGB()

        for point in points:
            #pointXYZRGB = gen_proto.Renderable_pb2.Renderable.PointXYZRGB()

            color = self.get_color_converted_point(point)
            gm.renderable.pointCloud.points.add()
            point_location = numpy.dot(pointcloud_transform, numpy.array([point[:3] + (1,)]).transpose())

            gm.renderable.pointCloud.points[-1].point.x = point_location[0,0]
            gm.renderable.pointCloud.points[-1].point.y = point_location[1,0]
            gm.renderable.pointCloud.points[-1].point.z = point_location[2,0]

            gm.renderable.pointCloud.points[-1].color.red = color[0]
            gm.renderable.pointCloud.points[-1].color.green = color[1]
            gm.renderable.pointCloud.points[-1].color.blue = color[2]

        gm.renderable.pointCloud.units = 1.0
        #ipdb.set_trace()
        if not self.graspit_socket.send_proto(gm):
            rospy.logwarn(str(self.__class__) + "::GraspitProtobufSocket:: Failed to send message")


    def create_uvs(self, pointcloud_msg):
        size_tuple = (pointcloud_msg.height, pointcloud_msg.width)
        if size_tuple != self.last_message_size:
            self.last_message_size = size_tuple
            self.uvs = [(u,v) for u in range(0, pointcloud_msg.height, self.downsample_factor)
                              for v in range(0, pointcloud_msg.width,  self.downsample_factor)]
        return

    def get_color_converted_point(self, point):

        if len(point) == 4:
            """
            Colors may come as a 3 8 bit integers BGR order for some reason some times. The message
            header is either parsed incorrectly by the point_cloud2 library or something funny is going on.
            """

            b, g, r = struct.unpack('BBBx', struct.pack('f', point[3]))
            color = [r/255.0, g/255.0, b/255.0]

        else:
            if len(point) == 6:
                color = point[3:]

        return color


if __name__ == "__main__":
    rospy.init_node('graspit_protobuf_socket')

    try:
        graspit_protobuf_socket_node = GraspitProtobufSocketNode()

        loop = rospy.Rate(10)

        while not rospy.is_shutdown():
            loop.sleep()
    except rospy.ROSInterruptException: pass