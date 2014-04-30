import graspit_protobuf_socket
import gen_proto.GraspitMessage_pb2
import gen_proto.Renderable_pb2
import sensor_msgs.point_cloud2
import sensor_msgs.msg
import rospy
import struct

class GraspitProtobufSocket(object):
    def __init__ (self, point_cloud_topic="pointcloud", graspit_socket = [], downsample_factor = 2, skip_msgs = 0):
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
        if downsample_factor:
            self.create_uvs()

    def relay_message(self, pointcloud_msg):
        """
        :type pointcloud_msg :sensor_msgs.msg.PointCloud2
        """
        if pointcloud_msg.header.seq % self.skip_msgs:
            return
        if self.downsample_factor:
            self.create_uvs(pointcloud_msg)
        points = sensor_msgs.point_cloud2.read_points(pointcloud_msg, True, self.uvs)
        colors = self.get_color_converted_points(self, points)
        gm = gen_proto.GraspitMessage_pb2.GraspitProtobufMessage()
        renderable = gen_proto.Renderable_pb2.Renderable()
        pointcloud = gen_proto.Renderable_pb2.Renderable.PointCloudXYZRGB()
        for point, color in zip(points, colors):
            pointXYZ = gen_proto.Renderable_pb2.Renderable.PointXYZ()
            pointXYZ.x = point[0]
            pointXYZ.y = point[1]
            pointXYZ.z = point[2]
            colorRGB = gen_proto.Renderable_pb2.Renderable.ColorRGB()
            colorRGB.r = color[0]
            colorRGB.g = color[1]
            colorRGB.b = color[2]

            pointXYZRGB = gen_proto.Renderable_pb2.Renderable.PointXYZRGB()
            pointXYZRGB.point = pointXYZ
            pointXYZRGB.color = colorRGB
            pointcloud.points.append(pointXYZRGB)
            pointcloud.units = 1.0
        renderable.pointCloud = [pointcloud]
        gm.renderable = renderable

        if not self.graspit_socket.send_proto(gm):
            rospy.logwarn(str(self.__class__) + "::GraspitProtobufSocket:: Failed to send message")


    def create_uvs(self, pointcloud_msg):
        size_tuple = (pointcloud_msg.height, pointcloud_msg.width)
        if size_tuple != self.last_message_size:
            self.last_message_size = size_tuple
            self.uvs = [(u,v) for u in range(0, pointcloud_msg.height, self.downsample_factor)
                              for v in range(0, pointcloud_msg.width,  self.downsample_factor)]
        return

    @staticmethod
    def get_color_converted_points(points):
        colors = []
        if points:
            if len(points[0]) == 4:
                """
                Colors may come as a 3 8 bit integers BGR order for some reason some times. The message
                header is either parsed incorrectly by the point_cloud2 library or something funny is going on.
                """
                colors = list()

                for point in points:
                    b, g, r = struct.unpack('BBBx', struct.pack('f',point[3]))
                    colors.append([r/255.0, g/255.0, b/255.0])

            else:
                if len(points[0]) == 6:
                    for point in points:
                        colors.append(point[3:])

        return colors
