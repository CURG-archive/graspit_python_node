

class BaseService(object):
    def __init__(self,*args):
        self.count = 0

    def run(self,request,reply):
        print("received " + self.__class__.__name__+" request " + str(self.count))
        self.count += 1
        response = self.build_response(request)
        reply.send(response)

