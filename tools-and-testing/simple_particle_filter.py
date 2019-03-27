from abc import ABCMeta, abstractmethod

import numpy as np
from random import random

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

for i in range(0, 100):
	filter.measureParticles([43, 43])
	filter.calculateWeights()
	filter.resample()
	print(filter.predict())
	# for particle in filter.particles:
	# 	print particle.pos
	filter.update(None)