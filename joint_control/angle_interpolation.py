'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from scipy.interpolate import CubicSpline
import numpy as np
import copy

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.is_running=False

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        keyframes_co = copy.deepcopy(keyframes)
        if not self.is_running:
            self.init_time=perception.time
        
        for i,_ in enumerate(keyframes_co[2]):
            for  n,_ in enumerate(keyframes_co[2][i]):
                keyframes_co[2][i][n]=keyframes_co[2][i][n][0]

        if len(keyframes_co[1])==0 or perception.time-self.init_time>max([max(i) for i in keyframes_co[1]]):
             self.is_running=False
             return {}

        self.is_running=True

        def find_local_maximas(ys):
            l=[0]
            y_old=ys[1]
            y_old_old=ys[0]
            for f,y in enumerate(ys[2:]):
                if (y-y_old)*(y_old-y_old_old)<0:
                    l.append(f+1)
                
                y_old_old=y_old
                y_old=y
            l.append(len(ys)-1)
            return l
        def get_css(x,y,lms):
            
            css=[]
            for i,lm in enumerate(lms[1:]):
                css.append(CubicSpline(x[lms[i]:lm+1],y[lms[i]:lm+1],bc_type=((1, 0.0), (1, 0.0))))
            return css
        def get_sp(x_train,lms,css,x_test):
            for p,i in enumerate(list(np.asarray(x_train)[lms][1:])):
                if x_test<i:
                    
                    return css[p](x_test)

            return css[-1](x_test)
        def inter(x_train,y_train,x_test):
            lms=find_local_maximas(y_train)
            css=get_css(x_train,y_train,lms)
            return get_sp(x_train,lms,css,x_test)



        for (x,y,name) in zip(keyframes_co[1],keyframes_co[2],keyframes_co[0]):
            #cs=CubicSpline(x,y,bc_type=((1, 0.0), (1, 0.0)))
            target_joints[name]=inter(x,y,perception.time-self.init_time)
        
        try:
            target_joints["RHipYawPitch"]=target_joints["LHipYawPitch"]
        except KeyError:
            pass
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello.hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
