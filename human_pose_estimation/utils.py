import numpy as np
import tensorflow as tf
import scipy

def mat2np(trajPath):

	'''converts text file output from matlab matrix to np array'''
	
	numJoints = 9

	traj = np.loadtxt(open(trajPath, "rb"), delimiter=",")

	trajPts = np.shape(traj)[0] #points per trajectory
	numTraj = np.shape(traj)[1]//numJoints #number of total trajectories

	#reshape traj data into 3d numpy array
	t = np.zeros([trajPts,numJoints,numTraj])
	for j in range(np.shape(traj)[0]):
		for i in range(np.shape(traj)[1]//numJoints):
			t[j,:,i] = traj[j,numJoints*i:numJoints*(i+1)]

	#swap axis so batch size is first axis (for TF)
	t = np.swapaxes(t,0,2)
	#swap axis again so that conv1D moves on time and not xyz
	t = np.swapaxes(t,1,2)

	rand = int(np.floor(np.random.rand()*numTraj))
	arr = t[rand,:,:]

	return arr