import rpcz
from gen_proto import run_recognition_pb2
from gen_proto import run_recognition_rpcz



if __name__ == "__main__":
    app = rpcz.Application()    
    stub = run_recognition_rpcz.ObjectRecognitionService_Stub(
        app.create_rpc_channel("tcp://localhost:5561"), "ObjectRecognitionService")

    request = run_recognition_pb2.ObjectRecognitionRequest()
    stub.run(request, deadline_ms=1000)
