from abc import ABCMeta, abstractmethod

import numpy as np
from random import random
import matplotlib.pyplot as plt

import sys
sys.path.append("..")
from particle_filter import *

window = (1, 1, 1)
circSize = window[0] / 25
actual = [0.43, 0.43, 0.43]

import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

class PFViz():
	def __init__(self, pf, frame, name, queue_size=10):
		self.pf = pf
		self.frame = frame
		self.name = name
		self.queue_size = queue_size
		#
		self.pub  = rospy.Publisher(self.name, PointCloud, queue_size=self.queue_size)
		rospy.init_node(self.name)
		#
	def update(self):
		pc = PointCloud()
		#
		pc.header.stamp = rospy.Time.now()
		pc.header.frame_id = self.frame
		#
		pc.channels = []
		for particle in self.pf.particles:
			pc.points.append(Point32(particle.pos[0], particle.pos[1], particle.pos[2]))
			#
		self.pub.publish(pc)

class myParticle(Particle):
	def __init__(self, pos=[0, 0, 0]):
		self.pos = [0, 0, 0]
		self.error = 0
		self.weight = 0

	def randomize(self):
		self.pos = [random()*2*window[0]-window[0], random()*2*window[1]-window[1], random()*2*window[2]-window[2]]

	def setPrediction(self, prediction):
		self.pos = prediction[:]

	def getPrediction(self):
		return self.pos

	def addNoise(self, noiseFactor): # Noisify? XD
		self.pos[0] = self.pos[0] + (random()*2*noiseFactor - noiseFactor)
		self.pos[1] = self.pos[1] + (random()*2*noiseFactor - noiseFactor)
		self.pos[2] = self.pos[2] + (random()*2*noiseFactor - noiseFactor)

	def setError(self, error):
		self.error = error

	def getError(self):
		return self.error

	def setWeight(self, weight):
		self.weight = weight

	def getWeight(self):
		return self.weight

	def update(self, newData):
		pass

def dist2(v1, v2):
	return (v1[0]-v2[0])**2 + (v1[1]-v2[1])**2 + (v1[2]-v2[2])**2

def myMetric(particle, observation):
	prediction = particle.getPrediction()
	return dist2(prediction, observation)

filter = ParticleFilter(500, myParticle, myMetric, 0.05, float(window[0])/100.0)
filter.generateParticles()

viz = PFViz(filter, "/odom", "myViz")

plt.ion()
fig, ax = plt.subplots()
x, y = [], []
scatterObj = ax.scatter(x, y) # Returns a tuple of line objects, thus the comma
plt.xlim(-window[0], window[0])
plt.ylim(-window[1], window[1])

circle1 = plt.Circle((43, 43), circSize, color='r')
circle2 = plt.Circle((0, 0), circSize, color='g')
ax.add_patch(circle1)
ax.add_patch(circle2)

plt.draw()

# Wait for a mouse click
while plt.waitforbuttonpress():
	pass

# while True:
for _ in range(0, 100):
	filter.measureParticles(actual)
	filter.calculateWeights()
	filter.resample()
	filter.predict()
	prediction = filter.predict()
	circle2.center = tuple(prediction)
	# for particle in filter.particles:
	# 	print particle.pos
	filter.update(None)

	print("Error: %d" % dist2(prediction, actual))

	x, y = [], []
	for particle in filter.particles:
		x.append(particle.pos[0])
		y.append(particle.pos[1])

	scatterObj.set_offsets(np.c_[x,y])
	fig.canvas.draw_idle()
	viz.update()
	plt.pause(0.0001)

# Wait for a mouse click
while plt.waitforbuttonpress():
	pass