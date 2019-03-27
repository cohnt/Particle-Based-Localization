from abc import ABCMeta, abstractmethod

import numpy as np
from random import random
import matplotlib.pyplot as plt

import sys
sys.path.append("..")
from particle_filter import *

window = (100, 100)
circSize = window[0] / 25
actual = [43, 43]

class myParticle(Particle):
	def __init__(self, pos=[0, 0]):
		self.pos = [0, 0]
		self.error = 0
		self.weight = 0

	def randomize(self):
		self.pos = [random()*2*window[0]-window[0], random()*2*window[1]-window[1]]

	def setPrediction(self, prediction):
		self.pos = prediction[:]

	def getPrediction(self):
		return self.pos

	def addNoise(self, noiseFactor): # Noisify? XD
		self.pos[0] = self.pos[0] + (random()*2*noiseFactor - noiseFactor)
		self.pos[1] = self.pos[1] + (random()*2*noiseFactor - noiseFactor)

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
	return (v1[0]-v2[0])**2 + (v1[1]-v2[1])**2

def myMetric(particle, observation):
	prediction = particle.getPrediction()
	return dist2(prediction, observation)

filter = ParticleFilter(500, myParticle, myMetric, 0.05, float(window[0])/100.0)
filter.generateParticles()

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
# while True:
for _ in range(0, 50):
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
	plt.pause(0.0001)

plt.waitforbuttonpress()