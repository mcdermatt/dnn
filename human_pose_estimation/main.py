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

#This file should take in some long trajectory (>5s) and make estimates of joint 
#		configurations at various points along the trajectory

#TODO
#	Train dataset on MOVING START!!!
#	make multiple estimates of human position from a single trajectory (10 pts/sec for n secs)
#	string together multiple estimates into "estimate trajectory"
#	be able to update viz with new trajectory information without restarting script
#	display an arbitrary number of estimates at once
#	draw actual and estimate in side by side windows?
#	make estimates with overlapping data (ex: 10 estimates using 50 points)

#load pretrained model
model = tf.keras.models.load_model("10DOF.kmod")

#import 10 second trajectory
ft1 = "simulation/data/traj_9DOF_long_sin.txt"
ft2 = "simulation/data/jointPos_9DOF_long_sin.txt"
ft3 = "simulation/data/jointPath_long_sin.txt"
numTraj = 11 #number of trajectories given in base file
runLen = 10 #1s @ 10pt/sec
tTest, jointPosTest = add_body_rotation(ft1, ft2, numTraj, actual_traj=ft3, numPts = runLen, training = False)
#setting training = False means every 1s incrament will be given the same rotation to represent real life trajectory

x_test = tf.convert_to_tensor(tTest, np.float32)
print("x_test shape", np.shape(x_test)) #(1,100,6) on single long trajectory, (3, 10, 6) on split multiple trajectories

# #for fixed number of estimates (one every 1s)
# estimate = np.zeros([(runLen//10),10])
# for i in range(runLen//10):
# 	estimate[i,:] = model.predict(x_test[:, (10*i):(10*(i+1)),:])

#for estimates only on the onset of motion (hand starts at zero velocity) THIS IS HOW THE MODEL WAS TRAINED!!
estimate = model.predict(x_test)
print("estimate = ",estimate)

#for overlapping estimates
# estimate = np.zeros([(runLen-10),10])
# for i in range(runLen-10):
# 	estimate[i,:] = model.predict(x_test[:, i:(i+10),:])

#TODO: some type of moving average filter to smooth out estimates!!!!

#this is the actual configuration of the human that we are trying to figure out
actual_joint_trajectory = np.load("simulation/data/at.npy")

#how the DNN thinks the human is configured #TODO: eventually predict this within this file
# estimate = np.load("simulation/data/prediction.npy")

viz = viz(actual_joint_trajectory, estimate, use_GPU=True, runFromMain = True)
viz.start()

#pressing F can break out of loop (so we can change up trajectories and stuff)
# print("ooowwwweee")
# viz.est = -estimate #works!
# viz.start()
