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


class GraspitProtobufSocketNode(object):
    def __init__(self, point_cloud_topic="/camera/depth_registered/points", graspit_socket=None, downsample_factor=1,
                skip_msgs=10, graspit_frame='/world'):
        """
        :type point_cloud_topic: str
        :type graspit_socket: graspit_protobuf_socket.GraspitProtobufSocket
        """
        self.subscriber = rospy.Subscriber(point_cloud_topic, sensor_msgs.msg.PointCloud2, self.relay_message,
                                           queue_size=1)
        if not graspit_socket:
            graspit_socket = graspit_protobuf_socket.GraspitProtobufSocket()
        self.graspit_socket = graspit_socket
        self.uvs = []
        self.last_message_size = (0, 0)
        self.skip_msgs = skip_msgs
        self.downsample_factor = downsample_factor
        self.graspit_frame = graspit_frame
        self.listener = tf.TransformListener()
        self.listener.setUsingDedicatedThread(True)

        rospy.loginfo(self.__class__.__name__ + " is inited")

    def add_frame_to_proto(self, frame_id, proto_msg):
        pose_msg = None
        try:
            self.listener.waitForTransform(self.graspit_frame, frame_id, rospy.Time(0), rospy.Duration(100))

            pose_msg = tf_conversions.toMsg(tf_conversions.fromTf(self.listener.lookupTransform(self.graspit_frame,
                                                                                                frame_id,
                                                                                              rospy.Time(0))))

        except Exception as e:
            rospy.logerr(self.__class__.__name__ + "::" +
                             "Failed to lookup frame transform from %s to %s -- %s"%(self.graspit_frame,
                                                                                     frame_id, e.message))
        if pose_msg:
            frame_proto = proto_msg.renderable.frame
            frame_proto.frame_id = frame_id
            frame_proto.orientation.w = pose_msg.orientation.w
            frame_proto.orientation.x = pose_msg.orientation.x
            frame_proto.orientation.y = pose_msg.orientation.y
            frame_proto.orientation.z = pose_msg.orientation.z

            frame_proto.translation.x = pose_msg.position.x
            frame_proto.translation.y = pose_msg.position.y
            frame_proto.translation.z = pose_msg.position.z
            frame_proto.units = 1.0

            proto_msg.renderable.renderableFrame = frame_id.strip('/')

        return proto_msg

    def make_pointcloud_proto(self, pointcloud_msg):
        """
        :param pointcloud_msg: input sensor_msgs.msg.PointCloud2
        :type pointcloud_msg: sensor_msgs.msg.PointCloud2
        :rtype: gen_proto.GraspitMessage_pb2.GraspitProtobufMessage
        """
        pointcloud_transform = tf_conversions.toMatrix(tf_conversions.fromTf(self.listener.lookupTransform(
            self.graspit_frame, pointcloud_msg.header.frame_id, rospy.Time(0))))
        rospy.loginfo(self.__class__.__name__ + " is relaying message")
        if self.downsample_factor != 1:
            self.create_uvs(pointcloud_msg)
        points = sensor_msgs.point_cloud2.read_points(pointcloud_msg, None, False, self.uvs)
        #points = sensor_msgs.point_cloud2.read_points(pointcloud_msg)

        gm = gen_proto.GraspitMessage_pb2.GraspitProtobufMessage()
        #gm.renderable.pointCloud.points.add()
        gm.renderable.pointCloud.points.extend(
            [gen_proto.Renderable_pb2.Renderable.PointXYZRGB()] *
            (pointcloud_msg.width*pointcloud_msg.height/(self.downsample_factor**2)))


        #renderable = gen_proto.Renderable_pb2.Renderable()
        #pointcloud = gen_proto.Renderable_pb2.Renderable.PointCloudXYZRGB()
        for ind,point in enumerate(points):
            #pointXYZRGB = gen_proto.Renderable_pb2.Renderable.PointXYZRGB()

            color = self.get_color_converted_point(point)

            point_location = numpy.dot(pointcloud_transform, numpy.array([point[:3] + (1,)]).transpose())
            renderable = gm.renderable
            assert isinstance(renderable, gen_proto.Renderable_pb2.Renderable)
            pointCloud = renderable.pointCloud
            assert isinstance(pointCloud, gen_proto.Renderable_pb2.Renderable.PointCloudXYZRGB)
            points = pointCloud.points
            assert isinstance(points, gen_proto.Renderable_pb2.Renderable.PointXYZRGB)
            pointCloud.points[ind].point.x = point_location[0, 0]
            pointCloud.points[ind].point.y = point_location[1, 0]
            pointCloud.points[ind].point.z = point_location[2, 0]

            pointCloud.points[ind].color.red = color[0]
            pointCloud.points[ind].color.green = color[1]
            pointCloud.points[ind].color.blue = color[2]

        renderable.pointCloud.units = 1.0

        rospy.loginfo(str(self.__class__) + "::GraspitProtobufSocket:: Finished building protobuf"
                                          + "pointcloud message")
        return gm


    def make_pointcloud2_proto(self, pointcloud_msg):
        """
        :param pointcloud_msg: input sensor_msgs.msg.PointCloud2
        :type pointcloud_msg: sensor_msgs.msg.PointCloud2
        :rtype: gen_proto.GraspitMessage_pb2.GraspitProtobufMessage
        """
        assert isinstance(pointcloud_msg, sensor_msgs.msg.PointCloud2)
        gm = gen_proto.GraspitMessage_pb2.GraspitProtobufMessage()
        gm = self.add_frame_to_proto(pointcloud_msg.header.frame_id, gm)
        pc2 = gm.renderable.pointCloud2

        assert isinstance(pc2, gen_proto.Renderable_pb2.Renderable.PointCloud2)
        pc2.id = pointcloud_msg.header.seq
        pc2.data = pointcloud_msg.data
        field_names = [field.name for field in pointcloud_msg.fields]
        pc2.fields.extend(field_names)
        pc2.height = pointcloud_msg.height
        pc2.width = pointcloud_msg.width
        pc2.is_bigendian = pointcloud_msg.is_bigendian
        pc2.is_dense = pointcloud_msg.is_dense
        pc2.units = 1.0
        return gm

    def relay_message(self, pointcloud_msg):
        """
        :param pointcloud_msg: input sensor_msgs.msg.PointCloud2
        :type pointcloud_msg: sensor_msgs.msg.PointCloud2
        """

        if pointcloud_msg.header.seq % self.skip_msgs:
            return
        try:
            if 0:
                gm = self.make_pointcloud_proto(pointcloud_msg)
            if 1:
                gm = self.make_pointcloud2_proto(pointcloud_msg)
            #ipdb.set_trace()
            if not self.graspit_socket.send_proto(gm):
                rospy.logwarn(str(self.__class__) + "::GraspitProtobufSocket:: Failed to send message")

        except Exception as e:
                rospy.logwarn(str(self.__class__) +
                              "::GraspitProtobufSocket:: Exception when trying to send message: %s"%e.message)
        finally:
            del gm

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