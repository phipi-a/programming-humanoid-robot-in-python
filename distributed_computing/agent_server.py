'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import numpy as np
import time
import sys
from jsonrpclib.SimpleJSONRPCServer import SimpleJSONRPCServer
from jsonrpclib.SimpleJSONRPCServer import SimpleJSONRPCServer





sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
import threading
from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.perception.joint[joint_name]
        
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.target_joints.update({joint_name:angle})
        

    def get_posture(self):
        '''return current posture of robot'''
        return self.posture
        

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        
        self.keyframes=keyframes
        while self.keyframes==keyframes:
            time.sleep(0.5)
        return

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.transforms[name].tolist()
        

    def set_transform(self, effector_name, transform):
        self.set_transforms(effector_name,np.asarray(transform),0)
        '''solve the inverse kinematics and control joints use the results
        '''
        
    def start_server(self,port):
        server = SimpleJSONRPCServer(('localhost', port))
        server.register_function(self.get_angle)
        server.register_function(self.set_angle)
        server.register_function(self.get_posture)
        server.register_function(self.get_transform)
        server.register_function(self.set_transform)
        server.register_function(self.execute_keyframes)
        
        print("Start server")
        t1 = threading.Thread(target=server.serve_forever)
        t1.daemon = True
        t1.start()
if __name__ == '__main__':
    agent = ServerAgent()
    agent.start_server(8888)
    agent.run()
    
