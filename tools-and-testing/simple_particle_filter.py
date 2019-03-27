from abc import ABCMeta, abstractmethod

import numpy as np
from random import random
import matplotlib.pyplot as plt

import sys
sys.path.append("..")
from particle_filter import *

class myParticle(Particle):
	def __init__(self, pos=[0, 0]):
		self.pos = [0, 0]
		self.error = 0
		self.weight = 0

	def randomize(self):
		self.pos = [random()*200-100, random()*200-100]

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

def myMetric(particle, observation):
	prediction = particle.getPrediction()
	return (prediction[0]-observation[0])**2 + (prediction[1]-observation[1])**2

filter = ParticleFilter(100, myParticle, myMetric, 0.05, 5)
filter.generateParticles()


# plt.ion()
# fig, ax = plt.subplots()
# x, y = [],[]
# sc = ax.scatter(x,y)
# plt.xlim(0,10)
# plt.ylim(0,10)

# plt.draw()
# for i in range(1000):
#     x.append(np.random.rand(1)*10)
#     y.append(np.random.rand(1)*10)
#     sc.set_offsets(np.c_[x,y])
#     fig.canvas.draw_idle()
#     plt.pause(0.1)

# plt.waitforbuttonpress()

plt.ion()
fig, ax = plt.subplots()
x, y = [], []
scatterObj = ax.scatter(x, y) # Returns a tuple of line objects, thus the comma
plt.xlim(-100, 100)
plt.ylim(-100, 100)

plt.draw()
while True:
	filter.measureParticles([43, 43])
	filter.calculateWeights()
	filter.resample()
	print(filter.predict())
	# for particle in filter.particles:
	# 	print particle.pos
	filter.update(None)

	x, y = [], []
	for particle in filter.particles:
		x.append(particle.pos[0])
		y.append(particle.pos[1])

	scatterObj.set_offsets(np.c_[x,y])
	fig.canvas.draw_idle()
	plt.pause(0.001)