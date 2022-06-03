'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH


from cmath import pi
from math import atan2
from numpy.matlib import matrix, identity
import numpy as np
from numpy import cos, sin, arcsin
import os
import sys
sys.path.append(os.path.join(os.path.abspath(
    os.path.dirname(__file__)), '..', 'joint_control'))
sys.path.append(os.path.join(os.path.abspath(
    os.path.dirname(__file__)), '..', 'software_installation'))
from spark_agent import INVERSED_JOINTS


from keyframes.hello import hello
from recognize_posture import PostureRecognitionAgent

 

def from_trans(m):
    return [m[0, -1], m[1, -1], m[2, -1], atan2(m[2,1],m[2,2]), atan2(-m[2,0],np.sqrt(m[2,1]**2+m[2,2]**2)), atan2(m[1,0],m[0,0])]


def rad2deg(x):
    return x*180/np.pi
def deg2rad(x):
    return x*np.pi/180
class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(
            simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'], 
                        'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'], 
                        'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                        'RArm': ['RShoulderPitch', 'RShoulderRoll','RElbowYaw', 'RElbowRoll' ], 
                        'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
                       
                       }
        

        self.rotation_axis = {'HeadYaw': ('z',0,0,126,-2.0857,2.0857), 'HeadPitch': ('y',0,0,0,-0.6720, 0.5149), 
                            'RShoulderPitch': ('y',0,-98,100,-2.0857,2.0857),'RShoulderRoll': ('z',0,0,0,-1.3265,0.3142), 'RElbowYaw': ('x',105,-15,0,-2.0857, 2.0857), 'RElbowRoll': ('z',0,0,0,0.0349, 1.5446), 
                            'LShoulderPitch': ('y',0,98,100,-2.0857,2.0857),'LShoulderRoll': ('z',0,0,0,-0.3142,1.3265), 'LElbowYaw': ('x',105,-15,0,-2.0857, 2.0857), 'LElbowRoll': ('z',0,0,0,0.0349, 1.5446), 
                            'LHipYawPitch': ('y,z',0,50,-85,-1.145303, 0.740810), 'LHipRoll': ('x',0,0,0,	-0.379472, 0.790477), 'LHipPitch': ('y',0,0,0,-1.535889, 0.484090), 'LKneePitch': ('y',0,0,-100,-0.092346, 2.112528), 'LAnklePitch': ('y',0,0,-102.9,-1.189516, 0.922747), 'LAnkleRoll': ('x',0,0,0,-0.397880, 0.769001),
                            'RHipYawPitch': ('y,z',0,-50,-85,-1.145303, 0.740810), 'RHipRoll': ('x',0,0,0,	-0.790477, 0.379472), 'RHipPitch': ('y',0,0,0,-1.535889, 0.484090), 'RKneePitch': ('y',0,0,-100,-0.103083, 2.120198), 'RAnklePitch': ('y',0,0,-102.9,-1.186448, 0.932056), 'RAnkleRoll': ('x',0,0,0,-0.768992, 0.397935)
                            }

        self.i=0
    
    def think(self, perception):
        
        self.forward_kinematics(perception.joint)
        # if self.i%100==0:
        #     np.set_printoptions(suppress=True)
        #     values=[np.round(from_trans(self.transforms[z]),decimals=2) for z in self.chains["LLeg"]]
        #     for i,_ in enumerate(values):
        #         values[i]=values[i][:3]
        #     print(*list(zip( self.chains["LLeg"],values)),sep='')
        #     print("\n")
        # self.i+=1
        return super(ForwardKinematicsAgent, self).think(perception)

    def getRotationMatrix(self, axis, angle,x,y,z):
        n = np.identity(4)
        r = angle
        if axis == 'x':
            n[0:3, 0:3] = np.asarray(
                [[1, 0, 0], [0, cos(r), -sin(r)], [0, sin(r), cos(r)]])
        elif axis == 'y':
            n[0:3, 0:3] = np.asarray(
                [[cos(r), 0, sin(r)], [0, 1, 0], [-sin(r), 0, cos(r)]])
        elif axis == 'z':
            n[0:3, 0:3] = np.asarray(
                [[cos(r), -sin(r), 0], [sin(r), cos(r), 0], [0, 0, 1]])
        n[:-1,-1]=[x,y,z]
        return n

    


    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        motion_axis,x,y,z,min,max = self.rotation_axis[joint_name]
        joint_angle=np.clip(joint_angle,min,max)

        if joint_name == 'LHipYawPitch':
            T = self.getRotationMatrix('x', deg2rad(45),0,0,0)
            T = T@self.getRotationMatrix('y', joint_angle,x,y,z)
        elif joint_name == 'RHipYawPitch':
            T = self.getRotationMatrix('x', deg2rad(-45),0,0,0)
            T = T@self.getRotationMatrix('y', joint_angle,x,y,z)
                
        else:
            T = self.getRotationMatrix(motion_axis, joint_angle,x,y,z)


        return T

    def forward_kinematics_own(self,effector_name,joints):
        T = identity(4)
        for joint in self.chains[effector_name]:
            angle = joints[joint]
            Tl = self.local_trans(joint,angle)
            T = T@Tl
        return T


    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():

            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint,angle)
                T = T@Tl
                self.transforms[joint] = T


if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    a=(["RShoulderRoll"],
    [[1,2]],
    [[[0, [],[]], [-1, [],[]]]])

    agent.keyframes = a
    agent.run()
