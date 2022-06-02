'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from ntpath import join
from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from numpy import arcsin, arccos, linalg,arctan2
from scipy.optimize import fmin, minimize

def from_trans(m):
    return np.asarray([m[0, -1], m[1, -1], m[2, -1], arctan2(m[2,1],m[2,2]), arctan2(-m[2,0],np.sqrt(m[2,1]**2+m[2,2]**2)), arctan2(m[1,0],m[0,0])])

def translate_rad(x):
    if x <0:
        x=(-1*x)%(2*np.pi)*-1
    else:
        x=x%(2*np.pi)
    if x>np.pi:
        x=(2*np.pi-x)*-1
    if x<-np.pi:
        x=x-2*np.pi*-1
    return x
def rad2deg(x):
    return np.round(x*180/np.pi,decimals=1)
def deg2rad(x):
    return x*np.pi/180
class InverseKinematicsAgent(ForwardKinematicsAgent):

    
    def error_func(self,joint_angles,joint_names, target,effector_name):
        Ts=self.forward_kinematics_own(effector_name,dict(zip(joint_names, joint_angles)))
        e = from_trans(target)[:3] - from_trans(Ts)[:3]
        self.a.append(linalg.norm(e))
        return linalg.norm(e)

    def inverse_kinematics(self, effector_name, transform,):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []

        target = transform
        if effector_name=="test":
            joint_names=self.chains_test[effector_name]
        else:
            joint_names=self.chains[effector_name]

        joint_names=joint_names

        joint_angles= [0]*len(joint_names)
        func = lambda t: self.error_func(t,joint_names, target,effector_name)
        
        #res=minimize(func, joint_angles, method='SLSQP')
        #joint_angles=res.x
        joint_angles= fmin(func, joint_angles)

        for i,n in enumerate(joint_names):
            #TODO clip angle
            _,_,_,_,min,max=self.rotation_axis[n]
            joint_angles[i]=np.clip(translate_rad(joint_angles[i]),min,max)

        joint_angles_dict = dict(zip(joint_names, joint_angles))
        

        # YOUR CODE HERE
        return joint_angles_dict

    def set_transforms(self, effector_name, transform,start_time):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.a=[]
        X=self.inverse_kinematics(effector_name,transform)
        print(X)
        print("Final Position:",from_trans(self.forward_kinematics_own(effector_name,X)))
        names=[]
        times=[]
        keys=[]
        for k,v in X.items():
            names.append(k)
            times.append([start_time,start_time+1])
            keys.append([[0.1,[0,0,0], [0,0,0]],[v,[0,0,0], [0,0,0]]])
        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    np.set_printoptions(suppress=True)
    # test inverse kinematics
    T = np.identity(4)
    T[:-1,-1]= np.asarray([0, 0.05, -0.26])*1000
    
    print("Target Position:",from_trans(T))
    agent.set_transforms('LLeg', T,0)
    
    agent.run()
