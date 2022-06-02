'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle
import sys,os

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control/robot_pose.pkl'),'rb'))  # LOAD YOUR CLASSIFIER
        self.classes = ['Left', 'StandInit', 'Frog', 'Crouch', 'HeadBack', 'Back', 'Sit', 'Knee', 'Stand', 'Right', 'Belly']
        self.features=['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        
        v=[perception.joint[joint_id] for joint_id in self.features]+[perception.imu[0],perception.imu[1]]
        posture=self.classes[self.posture_classifier.predict([v])[0]]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello.hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
