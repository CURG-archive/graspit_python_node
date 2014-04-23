
import rospy


class BaseService(object):
    def __init__(self, ros_interface):
        self.count = 0
        self.ros_interface = ros_interface

    def run(self, request, reply):
        rospy.loginfo("received request: " + str(request))
        rospy.loginfo("received " + self.__class__.__name__+" request " + str(self.count))
        self.count += 1
        response = self.build_response(request)
        reply.send(response)
        rospy.loginfo("sending response: " + str(response))


