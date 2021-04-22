import ctypes
import pyglet
from pyglet.gl import *
from pyglet.window import key
from pywavefront import visualization, Wavefront
import numpy as np
import time
from pyglet.window import mouse
from utils import *
from scipy.spatial.transform import Rotation as R

#TODO:
#	   Forward Kinematics

#			Make hand the base joint, place everything else relative to hand

#			Get distance between endpoint and shoulder


#	   Whole body rotation

#			estimate joint positions twice, figure out what human orientation would work to allow such successive joint angles

#			generate data normally, for each trial rotate all points about vertical y axis some random angle and save that angle as another joint angle?
#				Re-train network with an additional param?

class viz:
	"""Human visualization class made using OpenGL
	 .start() to run"""

	def __init__(self, truePath, pathBall, estimate, use_GPU = False):

		self.truePath = truePath
		self.pathBall = pathBall
		self.est = estimate
		print(self.est)
		self.lenPath = len(truePath)
		print("shape is" ,np.shape(self.truePath))

		if use_GPU is False:
			self.window = pyglet.window.Window(width=1280,height=720)
		else:
			config = pyglet.gl.Config(sample_buffers=1, samples=9) #samples = number of points used for AA
			self.window = pyglet.window.Window(width=1280, height=720, config = config)

		self.keys = key.KeyStateHandler()
		self.window.push_handlers(self.keys)
		self.window.push_handlers(self.on_mouse_drag)

		self.legs = Wavefront('simulation/assets/hipsAndLegs.obj')
		self.torso = Wavefront('simulation/assets/torso.obj')
		self.leftArm = Wavefront('simulation/assets/leftArm.obj')
		self.upperArm = Wavefront('simulation/assets/upperArm.obj')
		self.lowerArm = Wavefront('simulation/assets/lowerArm.obj')
		self.hand = Wavefront('simulation/assets/hand.obj')
		self.head = Wavefront('simulation/assets/head.obj')
		self.ball = Wavefront('simulation/assets/ball.obj')
		greenCheck = pyglet.image.load('simulation/assets/greenCheck.png')
		self.gc = pyglet.sprite.Sprite(img=greenCheck)
		self.gc.scale = 0.01
		self.gc.x = -10
		self.gc.y = 12
		redX = pyglet.image.load('simulation/assets/redX.png')
		self.rx = pyglet.sprite.Sprite(img=redX)
		self.rx.scale = 0.005
		self.rx.x = -10
		self.rx.y = 12
		grid = pyglet.image.load('simulation/assets/grid.png')
		self.grid = pyglet.sprite.Sprite(img = grid)
		self.grid.scale = 0.25

		self.l1 = 13.25
		self.l2 = 13.25
		self.l3 = 2.65
		self.rotation = 0
		self.cameraZ = 0.0
		self.i = 0 #count variable
		self.on_resize(1280,720)

		self.dx = 0
		self.dy = 0
		self.theta = 0
		self.dCam = 500
		self.label = None

		self.spf = 1/60 #seconds per frame

	def on_resize(self,width, height):
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(50., float(width)/height, 1., 1000.) #change first argument for fov #change last for max clip dist
		glTranslatef(0,-5,-50) #fits arm into camera view
		glMatrixMode(GL_MODELVIEW)
		return True

	def on_draw(self):
		self.window.clear()
		#set background color
		glClearColor(0.1,0.1,0.1,0.5) #night mode
		# glClearColor(1.,1.,1.,0.5) #white
		# glClearColor(0.69,1,0.69,0.5) #green

		glViewport(0,0,1280,720)
		glLoadIdentity()
		glMatrixMode(GL_PROJECTION)		

		if self.label:
			self.label.draw()
		glTranslatef(self.dx/20,self.dy/20,0)
		glRotatef(self.theta/5,0,1,0)
		#Zooming
		glScalef(abs(-self.dCam/500),abs(-self.dCam/500),abs(-self.dCam/500))

		glMatrixMode(GL_MODELVIEW)

		#draw ground plane
		glRotatef(90,1,0,0)
		glTranslatef(-100,-100,40)
		self.grid.draw()
		glRotatef(90,-1,0,0)
		glTranslatef(100,100,-40)

		lightfv = ctypes.c_float * 4

		#glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1.0*np.sin(rotation*0.1), 1.0, 0.0))
		glLightfv(GL_LIGHT0, GL_AMBIENT, lightfv(0.5,0.5,0.5,0.1))
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightfv(0.5, 0.5, 0.5, 0.6))
		glLightfv(GL_LIGHT0, GL_SPECULAR, lightfv(0.0,0.0,0.0,0.1))

		glEnable(GL_DEPTH_TEST)

		x = 0
		y = 0
		z = 0
		bodyRot = 0 #self.i/ 3
		j0 = -self.truePath[self.i,0] #self.i / 10
		j1 = -self.truePath[self.i,1] #self.i / 10
		j2 =  self.truePath[self.i,2] #self.i / 10
		
		j3 = self.truePath[self.i,3]  #chicken wing			#GOOD
		j4 = -self.truePath[self.i,4] # butterfly 				#GOOD
		j5 = -self.truePath[self.i,5]# forward arm raise  	#GOOD
		j6 = -self.truePath[self.i,6] #self.i #elbow
		j7 = -self.truePath[self.i,7] #self.i / 5 #wrist twist
		j8 = 45 + self.truePath[self.i,8] #30 + self.i / 2 #wrist in (shooting a basketball) - add 30 to start out straight


		#draw actual human position
		px, py, pz = self.human(x, y, z, j0, j1, j2, j3, j4, j5, j6, j7, j8, bodyRot, transparent = False, draw = True)
		#draw ball in hand of human
		self.draw_endpoint(px,py,pz, wireframe = False)

		#get coords of final position of actual human
		pxFinal, pyFinal, pzFinal = self.human(0,0,0, -self.truePath[-1,0], -self.truePath[-1,1], self.truePath[-1,2], self.truePath[-1,3],
										-self.truePath[-1,4], -self.truePath[-1,5], -self.truePath[-1,6], -self.truePath[-1,7],
										45 + self.truePath[-1,8],bodyRot = 0, draw = False)

		#get estimate of human configuration from network - run once without draw to get position of hand relative to base
		pxEst, pyEst, pzEst = self.human(0,0,0, -self.est[0], -self.est[1], self.est[2], self.est[3], -self.est[4], -self.est[5],
											-self.est[6], -self.est[7], 45 + self.est[8], bodyRot = 0, transparent=False, draw = False)

		#run a second time, translating so the hand of the human is located at the end of the ball trajectory
		self.human(pxFinal-pxEst, pyFinal-pyEst, pzFinal-pzEst, -self.est[0], -self.est[1], 
			self.est[2], self.est[3], -self.est[4], -self.est[5], -self.est[6], -self.est[7], 45 + self.est[8], bodyRot = 0, transparent=True, draw = True)

		#draw ball from file
		# self.draw_endpoint(self.pathBall[self.i%9,0],self.pathBall[self.i%9,1],self.pathBall[self.i%9,2]) #debug


		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE)
		
		time.sleep(0.01)

	def human(self, x, y, z, j0, j1, j2, j3, j4, j5, j6, j7, j8, bodyRot, transparent = False, draw = True):
		
		'''inputs: xyz of HIPS, all human joint angles, human body rotation'''

		wireframe = transparent

		self.draw_legs(x, y, z, bodyRot, wireframe, draw)
		shoulderx, shouldery, shoulderz = self.draw_torso(x, y, z, j0, j1, j2, bodyRot, wireframe, draw)
		elbowx, elbowy, elbowz = self.draw_upper_arm(shoulderx, shouldery, shoulderz, j0, j1, j2, j3, j4, j5, bodyRot, wireframe, draw)
		wristx, wristy, wristz = self.draw_lower_arm(elbowx, elbowy, elbowz, j0, j1, j2, j3, j4, j5, j6, bodyRot, wireframe, draw)
		palmx, palmy, palmz = self.draw_hand(wristx, wristy, wristz, j0, j1, j2, j3, j4, j5, j6, j7, j8, bodyRot, wireframe, draw)

		return palmx, palmy, palmz

	def draw_legs(self, x, y, z, bodyRot, wireframe = False, draw = True):
		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)
		glTranslatef(x, y, z)
		glRotatef(bodyRot,0,1,0) #[amount, x, y, z]??
		if wireframe:
			glPolygonMode( GL_FRONT, GL_POINT)
		if draw:
			visualization.draw(self.legs)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

	def draw_torso(self, x, y, z, j0, j1, j2, bodyRot, wireframe = False, draw = True):
		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)

		#shoulder to hips offset = [-7.8, 14.76, -2.1745]
		rbody = R.from_euler('y', bodyRot, degrees = True)
		r0 = R.from_euler('y', j0, degrees = True)
		r1 = R.from_euler('z', j1, degrees = True)
		r2 = R.from_euler('x', j2, degrees = True)

		base2shoulder = rbody*r0*r1*r2
		shoulderx, shouldery, shoulderz = base2shoulder.apply([-7.8, 14.76, -2.17])
		leftShoulderx, leftShouldery, leftShoulderz = base2shoulder.apply([7.8, 14.76, -2.17])
		headx, heady, headz = base2shoulder.apply([0.0, 22, -1.0])

		if wireframe:
			glPolygonMode( GL_FRONT, GL_POINT)

		#move body
		glTranslatef(x,y,z)

		#draw left arm
		glTranslatef(leftShoulderx, leftShouldery, leftShoulderz)
		glRotatef(bodyRot,0,1,0)
		glRotatef(j0, 0, 1, 0)
		glRotatef(j1, 0, 0, 1)
		glRotatef(j2, 1, 0, 0)
		if draw:
			visualization.draw(self.leftArm)
		glRotatef(-j2, 1, 0, 0)
		glRotatef(-j1, 0, 0, 1)
		glRotatef(-j0, 0, 1, 0)
		glRotatef(-bodyRot,0,1,0)
		glTranslatef(-leftShoulderx, -leftShouldery, -leftShoulderz)

		#draw head
		glTranslatef(headx, heady, headz)
		glRotatef(bodyRot,0,1,0)
		glRotatef(j0, 0, 1, 0)
		glRotatef(j1, 0, 0, 1)
		glRotatef(j2, 1, 0, 0)
		if draw:
			visualization.draw(self.head)
		glRotatef(-j2, 1, 0, 0)
		glRotatef(-j1, 0, 0, 1)
		glRotatef(-j0, 0, 1, 0)
		glRotatef(-bodyRot,0,1,0)
		glTranslatef(-headx, -heady, -headz)

		glTranslatef(shoulderx, shouldery, shoulderz)
		glRotatef(bodyRot,0,1,0)
		glRotatef(j0, 0, 1, 0)
		glRotatef(j1, 0, 0, 1)
		glRotatef(j2, 1, 0, 0)
		
		if draw:
			visualization.draw(self.torso)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

		return shoulderx+x, shouldery+y, shoulderz+z


	def draw_upper_arm(self, shoulderx, shouldery, shoulderz, j0, j1, j2, j3, j4, j5, bodyRot, wireframe = False, draw = True):
		
		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)

		glTranslatef(shoulderx, shouldery, shoulderz)
		glRotatef(bodyRot,0,1,0) #[amount, x, y, z]??
		glRotatef(j0, 0, 1, 0)
		glRotatef(j1, 0, 0, 1)
		glRotatef(j2, 1, 0, 0)

		glRotatef(j3, 0, 0, 1)
		glRotatef(j4, 0, 1, 0)
		glRotatef(j5, 1, 0, 0)

		#upper arm offset [-0.0 10.26 -0.9]
		if wireframe:
			glPolygonMode( GL_FRONT, GL_POINT)
		if draw:
			visualization.draw(self.upperArm)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

		#using rotation matrix------------------------------------- 
		rbody = R.from_euler('y', bodyRot, degrees = True)
		r0 = R.from_euler('y', j0, degrees = True)
		r1 = R.from_euler('z', j1, degrees = True)
		r2 = R.from_euler('x', j2, degrees = True)
		r3 = R.from_euler('z', j3, degrees = True) #chicken wing
		r4 = R.from_euler('y', j4, degrees = True) #butterfly motion
		r5 = R.from_euler('x', j5, degrees = True) #forwad arm raise

		shoulder2elbow = rbody*r0*r1*r2*r3*r4*r5
		dx, dy, dz = shoulder2elbow.apply([0.0, -10.26, -0.9])

		elbowx = dx + shoulderx
		elbowy = dy + shouldery
		elbowz = dz + shoulderz

		return elbowx, elbowy, elbowz

	def draw_lower_arm(self, elbowx, elbowy, elbowz, j0, j1, j2, j3, j4, j5, j6, bodyRot, wireframe = False, draw = True):
				#copypasta from draw_torso--------------------
		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)
		glTranslatef(elbowx,elbowy,elbowz)
		glRotatef(bodyRot,0,1,0) #[amount, x, y, z]??
		glRotatef(j0, 0, 1, 0)
		glRotatef(j1, 0, 0, 1)
		glRotatef(j2, 1, 0, 0)
		glRotatef(j3, 0, 0, 1)
		glRotatef(j4, 0, 1, 0)
		glRotatef(j5, 1, 0, 0)
		glRotatef(j6, -1, 0, 0) #TODO - debug this


		if wireframe:
			glPolygonMode( GL_FRONT, GL_POINT)
		if draw:
			visualization.draw(self.lowerArm)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

		#lower arm offset [-1.79 -10.55 0.61]
		rbody = R.from_euler('y', bodyRot, degrees = True)
		r0 = R.from_euler('y', j0, degrees = True)
		r1 = R.from_euler('z', j1, degrees = True)
		r2 = R.from_euler('x', j2, degrees = True)
		r3 = R.from_euler('z', j3, degrees = True) #chicken wing
		r4 = R.from_euler('y', j4, degrees = True) #butterfly motion
		r5 = R.from_euler('x', j5, degrees = True) #forwad arm raise
		r6 = R.from_euler('x', -j6, degrees = True) #elbow bend

		elbow2wrist = rbody*r0*r1*r2*r3*r4*r5*r6
		# dx, dy, dz = elbow2wrist.apply([1.79, -10.55, 0.61])
		dx, dy, dz = elbow2wrist.apply([0, -10.55, 0])

		wristx = elbowx + dx
		wristy = elbowy + dy
		wristz = elbowz + dz

		return wristx, wristy, wristz

	def draw_hand(self, wristx, wristy, wristz, j0, j1, j2, j3, j4, j5, j6, j7, j8, bodyRot, wireframe = False, draw = True):

		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)
		glTranslatef(wristx,wristy,wristz)
		glRotatef(bodyRot,0,1,0) #[amount, x, y, z]??
		glRotatef(j0, 0, 1, 0)
		glRotatef(j1, 0, 0, 1)
		glRotatef(j2, 1, 0, 0)
		glRotatef(j3, 0, 0, 1)
		glRotatef(j4, 0, 1, 0)
		glRotatef(j5, 1, 0, 0)

		glRotatef(j6, -1, 0, 0) #idk here
		glRotatef(j7, 0, 1, 0) #idk here
		glRotatef(j8, 0, 0, 1) #idk here


		if wireframe:
			glPolygonMode( GL_FRONT, GL_POINT)
		if draw:
			visualization.draw(self.hand)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

		rbody = R.from_euler('y', bodyRot, degrees = True)
		r0 = R.from_euler('y', j0, degrees = True)
		r1 = R.from_euler('z', j1, degrees = True)
		r2 = R.from_euler('x', j2, degrees = True)
		r3 = R.from_euler('z', j3, degrees = True) #chicken wing
		r4 = R.from_euler('y', j4, degrees = True) #butterfly motion
		r5 = R.from_euler('x', j5, degrees = True) #forwad arm raise
		r6 = R.from_euler('x', -j6, degrees = True) #elbow bend
		r7 = R.from_euler('y', j7, degrees = True)
		r8 = R.from_euler('z', j8, degrees = True)

		wrist2palm = rbody*r0*r1*r2*r3*r4*r5*r6*r7*r8
		dx, dy, dz = wrist2palm.apply([-1.5, -4, 0])

		palmx = wristx + dx
		palmy = wristy + dy
		palmz = wristz + dz

		return palmx, palmy, palmz

	def draw_endpoint(self, x, y, z, wireframe = False, draw = True):
		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)
		glTranslatef(x, y, z)
		if wireframe:
			glPolygonMode( GL_FRONT, GL_POINT)

		glScalef(2.,2.,2.)
		if draw:
			visualization.draw(self.ball)
		glScalef(1.,1.,1.)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

	# @window.event #doesen't work in class
	def on_mouse_drag(self,x,y,dx,dy,button,modifiers):
		"""move camera with mouse (click and drag)"""
		
		if button == pyglet.window.mouse.LEFT:
			self.dx += dx
			self.dy += dy

		if button == pyglet.window.mouse.RIGHT:
			self.theta += dx
			self.dCam += dy


	def update(self, dt):
		self.on_draw()
		self.on_resize(1280,720)

		self.i += 1
		if self.i == (self.lenPath - 1):
			self.i = 0 #loop

	def start(self):
		# pyglet.clock.schedule(self.update)
		pyglet.clock.schedule_interval(self.update, self.spf)
		pyglet.app.run()


if __name__ == "__main__":

	filename1 = "simulation/data/jointPath.txt"
	filename2 = "simulation/data/traj_9DOF_1.txt"

	#this is the actual configuration of the human that we are trying to figure out
	actual_joint_trajectory = mat2npJoints(filename1) #mat2npy only works for joints...

	#this is the trajectory of the ball that we are using to 
	endpoint_trajectory = mat2npEndpoint(filename2)[0]

	#how the DNN thinks the human is configured
	estimate = np.load("simulation/data/prediction.npy")[0]

	viz = viz(actual_joint_trajectory, endpoint_trajectory *10, estimate, use_GPU=True)

	viz.start()
