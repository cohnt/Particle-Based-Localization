from abc import ABCMeta, abstractmethod

import numpy as np

class Particle:
	__metaclass__ = ABCMeta
	
	@abstractmethod
	def __init__(self): # Must have default values for *all* arguments
		pass

	@abstractmethod
	def randomize(self):
		pass

	@abstractmethod
	def setPrediction(self, prediction):
		pass

	@abstractmethod
	def getPrediction(self):
		pass

	@abstractmethod
	def addNoise(self, noise): # Noisify? XD
		pass

	@abstractmethod
	def setError(self, error):
		pass

	@abstractmethod
	def getError(self):
		pass

	@abstractmethod
	def setWeight(self, weight):
		pass

	@abstractmethod
	def getWeight(self):
		pass

	@abstractmethod
	def update(self, newData):
		pass

class ParticleFilter:
	def __init__(self, numParticles, particleType, metric, explorationFactor=0):
		self.numParticles = numParticles
		self.particleType = particleType
		self.metric = metric

		try:
			_p = self.particleType()
		except TypeError
			print("Particle class %s must have default constructor %s()." % (self.particleType.__name__, self.particleType.__name__))
			raise(TypeError)
		self.particles = self.numParticles*[particleType()]

		self.explorationFactor = explorationFactor

	def generateParticles(self):
		for particle in self.particles:
			particle.randomize()

	def measureParticles(self, observation):
		for particle in self.particles:
			particle.setError(self.metric(particle, observation))

	def calculateWeights(self):
		error = []
		for particle in self.particles:
			error.append(particle.getError())
		error = np.array(error)

		v = np.var(error)
		m = np.mean(error)

		# Compute raw weights
		weights = np.zeros(error.size)
		for i in range(0, error.size):
			weights[i] = np.pow(np.e, -np.pow(error[i], 2)/(2*v)) * m

		# Normalize
		total = np.sum(weights)
		weights = np.divide(weights, total)

		# Store into the particle objects
		for particle, weight in zip(self.particles, weights):
			particle.setWeight(weight)

	def resample(self):
		weights = []
		for particle in self.particles
			weights.append(particle.getWeight())
		weights = np.array(weights)

		cs = np.cumsum(weights)
		step = 1/(self.numParticles * (1 - self.explorationFactor) + 1)
		chkVal = step
		chkIndex = 0
		newParticles = self.numParticles*[particleType()]

		for i in range(0, self.numParticles * (1 - self.explorationFactor)):
			while cs[chkIndex] < chkVal:
				chkIndex = chkIndex + 1
			chkVal = chkVal + step
			newParticles[i].setPrediction(self.particles[chkIndex].getPrediction())
			newParticles[i].addNoise(self.noiseFactor)

		for i in range(self.numParticles * (1 - self.explorationFactor), self.numParticles):
			newParticles[i].randomize()

		self.particles = newParticles

	def predict(self):
		predictions = []
		for i in range(0, self.particles * (1 - self.explorationFactor)):
			predictions.append(self.particles[i].getPrediction())
		predictions = np.array(predictions)
		return np.mean(predictions, axis=0)

	def update(self, newData):
		for particle in self.particles:
			particle.update(newData)