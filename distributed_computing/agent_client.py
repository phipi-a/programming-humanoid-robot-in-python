'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''
from jsonrpclib import Server
import weakref
import threading
import numpy as np
import sys
import time
import os
sys.path.append(os.path.join(os.path.abspath(
    os.path.dirname(__file__)), '..', 'joint_control'))
from keyframes import hello
class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        
        t=threading.Thread(target=self.proxy.execute_keyframes,args=[keyframes])
        
        t.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        t=threading.Thread(target=self.proxy.set_transform,args=[effector_name,transform])
        
        t.start()
        

class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    
    def __init__(self):
        self.post = PostHandler(self)
        self.conn = Server('http://localhost:8888')
        
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.conn.get_angle(joint_name)
        
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        
        self.conn.set_angle(joint_name,angle)

    def get_posture(self):
        '''return current posture of robot'''
        return self.conn.get_posture()
        

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        self.conn.execute_keyframes(keyframes)
        

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.conn.get_transform(name)
        

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.conn.set_transform(effector_name, transform)
        

if __name__ == '__main__':
    agent = ClientAgent()
    print("get_angle: ")
    print(agent.get_angle("HeadYaw"))
    print("set_angle: ")
    print(agent.set_angle("HeadYaw",-2))
    print("get_posture: ")
    print(agent.get_posture())
   
    print("get_transform: ")
    print(agent.get_transform("HeadYaw"))
    T = np.identity(4)
    T[:-1,-1]= np.asarray([0, 0.05, -0.26])*1000

    print("set_transform | blocking: ")
    print("-> blocking",agent.set_transform('LLeg', T.tolist()))
    print("execute_keyframes | blocking: ")
    print("-> blocking",agent.execute_keyframes(hello.hello()))
    
    print("set_transform | non-blocking: ")
    print("-> non-blocking",agent.post.set_transform('LLeg', T.tolist()))
    
    time.sleep(1)
    print("execute_keyframes | non-blocking: ")
    print("-> non-blocking", agent.post.execute_keyframes(hello.hello()))


