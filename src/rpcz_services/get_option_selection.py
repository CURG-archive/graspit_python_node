#!/usr/bin/env python
from gen_proto import get_option_selection_pb2
from gen_proto import get_option_selection_rpcz
from gen_proto import image_pb2

from base_service import BaseService
from rsvp_msgs.msg import RankAction, RankGoal
from sensor_msgs.msg import Image as ImageMsg, CompressedImage as CompressedImageMsg
import Image
from cStringIO import StringIO

import rospy
import rpcz
import actionlib
import threading

class OptionSelectionService(get_option_selection_rpcz.GetOptionSelectionService, BaseService):
    def __init__(self, ros_interface):
        super(OptionSelectionService, self).__init__(ros_interface)
        rospy.logwarn('GetOptionSelectionService initialized')
        self.client = actionlib.SimpleActionClient('rank', RankAction)
        self.client.wait_for_server()

    def build_response(self, request):
        rospy.logwarn('GetOptionSelectionService build response start: img {} cim {} str {}'.format(len(request.imageOptions), len(request.compressedImageOptions), len(request.stringOptions)))

        goal = RankGoal()

        for img_opt in request.imageOptions:
            if not img_opt.option.data:
                rospy.logwarn('image has no data, skipping!')
                continue
            msg = ImageMsg(height=img_opt.option.height,
                           width=img_opt.option.width,
                           encoding=image_pb2.PixelFormat.Name(img_opt.option.pixel_format),
                           step=img_opt.option.step,
                           data=img_opt.option.data)

            goal.option_ids.append(img_opt.description.id)

            goal.imgs.append(msg)
            # goal.descriptions.append(img_opt.description.description)
            # goal.costs.append(img_opt.description.cost)

        for c_img_opt in request.compressedImageOptions:
            rospy.logwarn('{}'.format(len(c_img_opt.option.data)))

            if not c_img_opt.option.data:
                rospy.logwarn('compressed image has no data, skipping!')
                continue

            Image.open(StringIO(c_img_opt.option.data)).save('img_{}.png'.format(c_img_opt.description.id), format='png')

            msg = CompressedImageMsg(format=c_img_opt.option.format,
                                     data=c_img_opt.option.data)

            goal.option_ids.append(c_img_opt.description.id)
            goal.compressed_imgs.append(msg)

            # goal.descriptions.append(c_img_opt.description.description)
            # goal.costs.append(c_img_opt.description.cost)

        for s_opt in request.stringOptions:
            goal.option_ids.append(s_opt.description.id)
            goal.strs.append(s_opt.option)

            # goal.descriptions.append(s_opt.description.description)
            # goal.costs.append(s_opt.description.cost)

        # goal.minimum_confidence_level = request.minimumConfidenceLevel

        rospy.logwarn('goal sent')
        self.client.send_goal(goal)

        response = get_option_selection_pb2.GetOptionSelectionResponse()

        self.client.wait_for_result(rospy.Duration.from_sec(120.0))

        result = self.client.get_result()
        rospy.logwarn('result received {}'.format(result))

        if result is not None and result.option_ids:
            response.selectedOption = result.option_ids[0]
            response.confidence = result.confidences[0]
            for c in result.confidences:
                response.interestLevel.append(c)
            for oid in result.option_ids:
                response.option_ids.append(oid)
        else:
            response.confidence = -1111
            rospy.logerr('THERE IS AN ERROR {}'.format(result))

        return response

class ServerThread(threading.Thread):
    def __init__(self, server_address, services):
        threading.Thread.__init__(self)
        self.daemon = True
        self.server_address = server_address
        self.services = services

    def run(self):
        app = rpcz.Application()
        server = rpcz.Server(app)

        for service in self.services:
            rospy.logwarn('registering service: {} as rpcz service: {}'.format(service.__class__.__name__, service.DESCRIPTOR.name))

            server.register_service(service, service.DESCRIPTOR.name)

        server.bind(self.server_address)
        rospy.logwarn('Serving requests on port {}'.format(self.server_address))
        app.run()


if __name__ == '__main__':
    rospy.init_node('rsvp_proxy')
    server_address = 'tcp://*:5561'
    services = (
        OptionSelectionService(),
    )

    rospy.logwarn('launching rsvp proxy server')
    rpcz_server = ServerThread(server_address, services)
    rpcz_server.start()

    loop = rospy.Rate(10)
    while not rospy.is_shutdown() and rpcz_server.is_alive():
        try:
            loop.sleep()
        except (KeyboardInterrupt, SystemExit):
            break
    rpcz_server.join(1)
