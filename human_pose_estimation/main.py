from viz import viz
import numpy as np
from utils import *
import pyglet
from pyglet.window import key, mouse

#This file should take in some long trajectory (>5s) and make estimates of joint 
#		configurations at various points along the trajectory

#TODO
#	be able to update viz with new trajectory information without restarting script
#	display an arbitrary number of estimates at once

filename1 = "simulation/data/at.npy" 	#ground truth movement traj
filename2 = "simulation/data/traj_9DOF_1.txt"	#

#this is the actual configuration of the human that we are trying to figure out
actual_joint_trajectory = np.load(filename1)

#this is the trajectory of the ball that we are using to make our estimate 
endpoint_trajectory = mat2npEndpoint(filename2)[0]

#how the DNN thinks the human is configured
estimate = np.load("simulation/data/prediction.npy")[0]

viz = viz(actual_joint_trajectory, endpoint_trajectory *10, estimate, use_GPU=True)
viz.start()

#pressing F can break out of loop (so we can change up trajectories and stuff)
print("ooowwwweee")
viz.est = -estimate #works!
viz.start()
