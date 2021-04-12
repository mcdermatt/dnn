import ctypes
import pyglet
from pyglet.gl import *
from pyglet.window import key
from pywavefront import visualization, Wavefront
import numpy as np
import time
from pyglet.window import mouse
from utils import mat2np

#TODO:
#	   Inverse Kinematics
#			Make hand the base joint, place everything else relative to hand

#	   Whole body rotation
#			estimate joint positions twice, figure out what human orientation would work to allow such successive joint angles

class viz:
	"""Human visualization class made using OpenGL
		(path( 3*n numpy array), use_GPU = False)
	 .start() to run"""

	def __init__(self, path,path2, use_GPU = False):

		self.pathA = path
		self.pathB = path2
		self.lenPath = len(path)
		# self.tp = trajPlotter(self.pathA,self.pathB)

		if use_GPU is False:
			self.window = pyglet.window.Window(width=1280,height=720)
		else:
			config = pyglet.gl.Config(sample_buffers=1, samples=9) #samples = number of points used for AA
			self.window = pyglet.window.Window(width=1280, height=720, config = config)

		self.keys = key.KeyStateHandler()
		self.window.push_handlers(self.keys)
		self.window.push_handlers(self.on_mouse_drag)

		self.base = Wavefront('simulation/assets/hipsAndLegs.obj')
		self.link0 = Wavefront('simulation/assets/torso.obj')
		self.link1 = Wavefront('simulation/assets/upperArm.obj')
		self.link1Clear = Wavefront('simulation/assets/upperArm.obj')
		self.link2 = Wavefront('simulation/assets/lowerArm.obj')
		self.link2Clear = Wavefront('simulation/assets/lowerArm.obj')
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
		self.grid.scale = 0.1

		self.l1 = 13.25
		self.l2 = 13.25
		self.l3 = 2.65
		self.rotation = 0
		self.cameraZ = 0.0
		self.i = 0
		self.on_resize(1280,720)

		self.dx = 0
		self.dy = 0
		self.theta = 0
		self.dCam = 0

		#test
		#TODO generate plot from here
		self.label = None
		# plotFigPath = "pathFig.png"
		# plotFig = pyglet.image.load(plotFigPath)


		# self.plotFig = pyglet.sprite.Sprite(img=plotFig)
		# self.plotFig.scale = 0.0375
		# self.plotFig.x = -32
		# self.plotFig.y = 5

		self.spf = 1/60 #seconds per frame

	def on_resize(self,width, height):
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(80., float(width)/height, 1., 1000.) #change first argument for fov #change last for max clip dist
		glTranslatef(0,-5,-50) #fits arm into camera view
		glMatrixMode(GL_MODELVIEW)
		return True

	def on_draw(self):
		self.window.clear()
		#set background color
		# glClearColor(0.1,0.1,0.1,0.5) #night mode
		glClearColor(1.,1.,1.,0.5) #white
		glViewport(0,0,1280,720)
		glLoadIdentity()
		glMatrixMode(GL_PROJECTION)		

		if self.label:
			self.label.draw()
		glTranslatef(self.dx/20,self.dy/20,0)
		glRotatef(self.theta/5,0,1,0)
		#TODO: Fix zooming
		# glTranslatef(np.sin(np.deg2rad(self.theta/5))*self.dCam/5,0,np.cos(np.deg2rad(self.theta/5))*self.dCam/5)

		glMatrixMode(GL_MODELVIEW)

		#draw ground plane
		glRotatef(90,1,0,0)
		glTranslatef(-50,-50,20)
		self.grid.draw()
		glRotatef(90,-1,0,0)
		glTranslatef(50,50,-20)

		link0RotA = (180/np.pi)*self.pathA[self.i,0]
		link1RotA = (180/np.pi)*self.pathA[self.i,1]
		link2RotA = (180/np.pi)*self.pathA[self.i,2]

		link0RotB = (180/np.pi)*self.pathB[self.i,0]
		link1RotB = (180/np.pi)*self.pathB[self.i,1]
		link2RotB = (180/np.pi)*self.pathB[self.i,2]

		lightfv = ctypes.c_float * 4

		link2RotEffA = link1RotA + link2RotA
		link2RotEffB = link1RotB + link2RotB

		xElbA = ( self.l1 * np.sin(link0RotA*(np.pi/180))*np.sin(link1RotA*(np.pi/180)))
		yElbA = ( self.l1 * np.cos((link1RotA*(np.pi/180)))) 
		zElbA =  ( self.l1 * np.cos(link0RotA*(np.pi/180))*np.sin(link1RotA*(np.pi/180)))

		xElbB = ( self.l1 * np.sin(link0RotB*(np.pi/180))*np.sin(link1RotB*(np.pi/180)))
		yElbB = ( self.l1 * np.cos((link1RotB*(np.pi/180)))) 
		zElbB =  ( self.l1 * np.cos(link0RotB*(np.pi/180))*np.sin(link1RotB*(np.pi/180)))


		#glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1.0*np.sin(rotation*0.1), 1.0, 0.0))
		# glLightfv(GL_LIGHT0, GL_AMBIENT, lightfv(0.5,0.5,0.5,0.1))
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightfv(0.5, 0.5, 0.5, 0.6))
		glLightfv(GL_LIGHT0, GL_SPECULAR, lightfv(0.0,0.0,0.0,0.1))

		glEnable(GL_DEPTH_TEST)

		# self.draw_base(self.base)
		# self.draw_link0(self.link0, 0, 0, 0, link0RotA)
		# self.draw_link1(self.link1, 0, 0, 0,link0RotA, link1RotA)
		# self.draw_link1(self.link1Clear, 0, 0, 0,link0RotB, link1RotB, wireframe=True)
		# self.draw_link2(self.link2, xElbA, yElbA, zElbA, link0RotA, link1RotA, link2RotA)
		# self.draw_link2(self.link2Clear, xElbB, yElbB, zElbB, link0RotB, link1RotB, link2RotB, wireframe=True)
		self.draw_endpoint(self.ball,link0RotB, link1RotB, link2RotB)

		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE)
		
		time.sleep(0.01)

	def draw_base(self,link):
		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)
		glRotatef(45,0,1,0)
		glTranslatef(0,-3.4,0)
		visualization.draw(link)

	def draw_link0(self,link, x, y, z, link0Rot):
		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)
		glRotatef(link0Rot, 0.0, 1.0 , 0.0)
		glTranslatef(x, y, z)

		visualization.draw(link)

	def draw_link1(self,link, x, y, z,link0Rot, link1Rot, wireframe=False):
		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)
		#glRotatef(180,0,1,0) #flips l1 around so it isnt bending backwards
		glRotatef(link0Rot, 0.0, 1.0 , 0.0)
		glRotatef(link1Rot, 1.0, 0.0 , 0.0)
		if wireframe:
			glPolygonMode( GL_FRONT, GL_POINT)
		visualization.draw(link)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
		
	def draw_link2(self,link, x, y, z, link0Rot, link1Rot, link2Rot, wireframe=False):
		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)
		glTranslatef(x, y, z)
		#print("link0Rot: ", link0Rot, " Link1Rot: ", link1Rot, " Link2Rot: ", link2Rot)
		glRotatef(link0Rot, 0.0, 1.0 , 0.0)
		glRotatef(link1Rot, 1.0, 0.0 , 0.0)
		glRotatef(link2Rot, 1.0, 0.0, 0.0)
		if wireframe:
			glPolygonMode( GL_FRONT, GL_POINT)
		visualization.draw(link)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

	def draw_endpoint(self, link, x, y, z, wireframe = False):
		glLoadIdentity()
		glMatrixMode(GL_MODELVIEW)
		glTranslatef(x, y, z)
		if wireframe:
			glPolygonMode( GL_FRONT, GL_POINT)
		visualization.draw(link)
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

	# filename1 = "best_path.npy"
	# filename2 = "best_goal_path.npy"

	filename1 = "simulation/data/traj_random1k.txt"
	filename2 = "simulation/data/traj_random1k.txt"

	path1 = mat2np(filename1)
	path2 = mat2np(filename2)

	# path1 = np.load(p1)
	# path2 = np.load(p2)

	# test = np.ones([np.shape(path1)[0],3])*np.array([1,0.5,1.2])

	viz = viz(path1, path2, use_GPU=True)

	viz.start()
