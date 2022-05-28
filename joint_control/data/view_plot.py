# load numpy array from npy file
from numpy import load
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# create empty lists for the x and y data
# x = []
# y = []

# # create the figure and axes objects
# fig, ax = plt.subplots()

# for i in range(100):
#     x=load('/home/philipp/uni/RoboCup/programming-humanoid-robot-in-python/joint_control/data/sensor.npy')
#     print(x)

#     ax.clear()
#     ax.plot(x)
#     ax.set_xlim([0,20])
#     plt.show()
#     time.sleep(0.5)

# # load array


# data = load('/home/philipp/uni/RoboCup/programming-humanoid-robot-in-python/joint_control/data/sensor.npy')

import matplotlib.pyplot as plt
import numpy as np



# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111)
x=load('/home/philipp/uni/RoboCup/programming-humanoid-robot-in-python/joint_control/data/sensor.npy')
z=load('/home/philipp/uni/RoboCup/programming-humanoid-robot-in-python/joint_control/data/target.npy')

line1, = ax.plot(x, 'r-') # Returns a tuple of line objects, thus the comma
line2, = ax.plot(z, 'b-') # Returns a tuple of line objects, thus the comma
ax.set_ylim([-1,1])
while True:
    try:
        x=load('/home/philipp/uni/RoboCup/programming-humanoid-robot-in-python/joint_control/data/sensor.npy')
        z=load('/home/philipp/uni/RoboCup/programming-humanoid-robot-in-python/joint_control/data/target.npy')

    except ValueError:
        continue
    line1.set_ydata(x)
    line2.set_ydata(z)
    try:
        fig.canvas.draw()
        fig.canvas.flush_events()
    except:
        pass
