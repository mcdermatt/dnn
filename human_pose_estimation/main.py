from viz import viz
import numpy as np
import tensorflow as tf
from utils import *
import pyglet
from pyglet.window import key, mouse
import time

#need to have these two lines to work on my ancient 1060 3gb
#  https://stackoverflow.com/questions/43990046/tensorflow-blas-gemm-launch-failed
physical_devices = tf.config.list_physical_devices('GPU') 
tf.config.experimental.set_memory_growth(physical_devices[0], True)

#load pretrained model
model = tf.keras.models.load_model("10DOF.kmod")

#import 10 second trajectory ----------------
#for linear force movement
ft1 = "simulation/data/traj_9DOF_long_linear.txt"
ft2 = "simulation/data/jointPos_9DOF_long_linear.txt"
ft3 = "simulation/data/jointPath_long_linear.txt"

#for sinusoidal movement
# ft1 = "simulation/data/traj_9DOF_long_sin.txt"
# ft2 = "simulation/data/jointPos_9DOF_long_sin.txt"
# ft3 = "simulation/data/jointPath_long_sin.txt"

numTraj = 11 #number of trajectories given in base file
runLen = 10 #1s @ 10pt/sec
tTest, jointPosTest = add_body_rotation(ft1, ft2, numTraj, actual_traj=ft3, numPts = runLen, training = False)
			#setting training = False means every 1s incrament will be given the same rotation to represent real life trajectory

x_test = tf.convert_to_tensor(tTest, np.float32)

#for estimates only on the onset of motion (hand starts at zero velocity) ----------------
estimate = model.predict(x_test)
#-----------------------------------------------------------------------------------------
#For testing: make up random estimates for joint angles (within range)-------------------
# estimate = np.random.rand(11,10)*[25., 30., 33.75, 55. , 60., 180., 65., 90., 55., 360.] + [0., 0., 26.25, -35., 30., 0., -65., 0., 0., -180.]
#----------------------------------------------------------------------------------------

#this is the actual configuration of the human that we are trying to figure out
actual_joint_trajectory = np.load("simulation/data/at.npy") #this file is created by add_body_rotation()

viz = viz(actual_joint_trajectory, estimate, use_GPU=True, runFromMain = True)
viz.start()