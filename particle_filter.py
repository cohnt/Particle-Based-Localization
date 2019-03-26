from abc import ABCMeta, abstractmethod

class Particle:
	__metaclass__ = ABCMeta
	
	@abstractmethod
	def __init__(self): # Must have default values for *all* arguments
		pass

	@abstractmethod
	def randomize(self):
		pass

	@abstractmethod
	def setWeight(self):
		pass

	@abstractmethod
	def getWeight(self):
		pass

	@abstractmethod
	def update(self):
		pass

class ParticleFilter:
	def __init__(self):
		pass

	def generateParticles(self):
		pass

	def measureParticles(self):
		pass

	def calculateWeights(self):
		pass

	def resample(self):
		pass

	def updateParticles(self):
		pass

	def predict(self):
		pass

	def update(self):
		pass