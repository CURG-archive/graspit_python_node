
from gen_proto import execute_grasp_pb2
from gen_proto import execute_grasp_rpcz

from base_service import BaseService

import utils


class ExecuteGraspService(execute_grasp_rpcz.ExecuteGraspService, BaseService):

    def __init__(self, ros_interface):
        super(ExecuteGraspService, self).__init__(ros_interface)

    def build_response(self, request):
        #utils.save_rpcz(request, "execute_grasp_request" + str(request.grasp.graspId) + str(".saved_proto"))
        grasp_msg = utils.build_grasp_msg(request.grasp)

        self.ros_interface.handle_execute_grasp_request(grasp_msg)

        rpcz_response = execute_grasp_pb2.ExecuteGraspResponse()
        rpcz_response.isSuccessful = True

        return rpcz_response
