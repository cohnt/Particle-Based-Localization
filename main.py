from detector import Detector
from particle_filter import ParticleFilter, Particle
from pf_viz import PFViz
from transformer import Transformer

import rospy
import numpy as np

from random import random

# TODO: Remove
from fake_detector import FakeDetector

environmentBounds = [[-2, 2], [-2, 2], [0, 2]]

class HParticle():
	def __init__(self, pos=[0, 0, 0]):
		self.pos = [0, 0, 0]
		self.error = 0
		self.weight = 0

	def randomize(self):
		self.pos = [
			random()*(environmentBounds[0][1]-environmentBounds[0][0])+environmentBounds[0][0],
			random()*(environmentBounds[1][1]-environmentBounds[1][0])+environmentBounds[1][0],
			random()*(environmentBounds[2][1]-environmentBounds[2][0])+environmentBounds[2][0]
		]

	def setPrediction(self, prediction):
		self.pos = prediction[:]

	def getPrediction(self):
		return self.pos

	def addNoise(self, noiseFactor):
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

def metric(particle, observation):
	# prediction = particle.getPrediction()
	# return dist2(prediction, observation)

	# TODO TODO TODO

	startPoint = observation[0]
	endPoints = observation[1]

	dists = []
	for i in range(0, len(endPoints)):
		d = np.linalg.norm(np.cross(endPoints[i]-startPoint, startPoint-particle.getPrediction()))/np.linalg.norm(endPoints[i]-startPoint)
		dists.append(d)

	return np.min(dists)

def main():
	rospy.init_node("particle_based_tracking")

	detector = FakeDetector("fake_detector")
	transformer = Transformer("transformer")
	filter = ParticleFilter(2500, HParticle, metric, 0.1, 0.01)
	filter.generateParticles()
	viz = PFViz(filter, "/odom", "myViz")

	while not rospy.is_shutdown():
		try:
			pixels = detector.fakeDetect()
			startPoint, endPoints = transformer.transform(pixels)
			filter.measureParticles((startPoint[:-1], np.asarray(endPoints)[:,:-1]))
			filter.calculateWeights()
			filter.resample()
			prediction = filter.predict()
			filter.update(None)
			viz.update()
		except KeyboardInterrupt:
			break

if __name__ == "__main__":
	main()