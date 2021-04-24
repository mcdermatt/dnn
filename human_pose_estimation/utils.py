import numpy as np
import tensorflow as tf
import scipy
from scipy.spatial.transform import Rotation as R

def mat2npJoints(trajPath):

	'''converts MatLab output text file of actual joint trajectories to np array'''
	
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


def mat2npEndpoint(file, numTraj = 2):

	'''converts text file output from matlab matrix to np array'''

	traj = np.loadtxt(open(file, "rb"), delimiter=",")

	trajPts = 10
			
	t = np.zeros([trajPts,6,numTraj]) #net 3
	for j in range(np.shape(traj)[0]):
	    for i in range(np.shape(traj)[1]//6):
	        t[j,:,i] = traj[j,6*i:6*(i+1)]

	#swap axis so batch size is first axis (for TF)
	t = np.swapaxes(t,0,2)
	# print(np.shape(t)) #[numTraj, xyz, trajPts]
	#swap axis again so that conv1D moves on time and not xyz
	t = np.swapaxes(t,1,2)

	return t

def add_body_rotation(endpoint_trajectory, joint_pos_file, numTraj):

	''' Endpoint trajectory data from SimScape Multibody simulation assumes 
		coordinate frame is relative to the hips of the human. This means that all
		data is generated with +x meaning to the left of the hips, +z forwards. 
		Obviously in the real world the human can be spun in any angle which drastically 
		changes the joint angle predictions. 

		This function rotates such trajectory data about the vertical (y) axis by some random
		angle and then saves this value as another DOF in the joint_pos_file for each trial

		**An added benefit of this is that these rotations can be use to artificially 
		  create more trainig data for a given number of simulation runs*** 

	Inputs: strings- file paths for traj and joint pos file locations

	#TODO - use this as a cheat to get more data

	'''
	numPts = 10 #number of individual points in each endpoint trajectory

	traj = mat2npEndpoint(endpoint_trajectory, numTraj)
	# print(np.shape(traj))
	joints = np.loadtxt(open(joint_pos_file, "rb"), delimiter=",")
	# print(np.shape(joints)[1])
	# print("joints ", joints)

	#joints with body rotation
	jbr = np.zeros([numTraj, 10])
	#trajectories with body rotation
	tbr = np.zeros([numTraj, numPts, 6])

	for i in range(numTraj):
		
		t = traj[i] #[x, y, z, thetax?, thetay?, thetaz?]
		
		#things get weird if only 1 trial is loaded
		try: 
			dummy = np.shape(joints)[1] #idk there's probably a cleaner way to do this
			j = joints[i]
		except:
			j = joints

		rotation = np.random.rand()*180 - 90
		rbody = R.from_euler('y', rotation, degrees = True)

		for m in range(numPts):			
			#ACCOUNT FOR EFFECT OF AUGMENTING DATA ON JOINT POSITIONS
			t[m,0], t[m,1], t[m,2] = rbody.apply([t[m,0], t[m,1], t[m,2]])

			#ACCOUNT FOR EFFECT OF AUGMENTING DATA ON ENDPOINT ANGLES 
			#	Actually don't do this- it gives away too much information!!?
			# t[j,4] = t[j,4] + rotation

		jbr[i,:9] = j
		jbr[i,9] = rotation #saves how much was rotated in the jointpos array

		tbr[i] = t

	# print(tbr)
	# print(jbr)

	np.save("simulation/data/tbr.npy", tbr)
	np.save("simulation/data/jbr.npy", jbr)

	print("saved augmented data as tbr, jbr")

	return tbr, jbr

