from detector import Detector
from particle_filter import ParticleFilter, Particle
from pf_viz import PFViz
from transformer import Transformer

import rospy
import numpy as np
import random
import time

environmentBounds = [[-2, 2], [-2, 2], [0, 2]]

numItersPerSample = 5
numHist = 5
randHist = True

class HParticle():
	def __init__(self, pos=[0, 0, 0]):
		self.pos = [0, 0, 0]
		self.error = 0
		self.weight = 0

	def randomize(self):
		self.pos = [
			random.random()*(environmentBounds[0][1]-environmentBounds[0][0])+environmentBounds[0][0],
			random.random()*(environmentBounds[1][1]-environmentBounds[1][0])+environmentBounds[1][0],
			random.random()*(environmentBounds[2][1]-environmentBounds[2][0])+environmentBounds[2][0]
		]

	def setPrediction(self, prediction):
		self.pos = prediction[:]

	def getPrediction(self):
		return self.pos

	def addNoise(self, noiseFactor):
		self.pos[0] = self.pos[0] + (random.random()*2*noiseFactor - noiseFactor)
		self.pos[1] = self.pos[1] + (random.random()*2*noiseFactor - noiseFactor)
		self.pos[2] = self.pos[2] + (random.random()*2*noiseFactor - noiseFactor)

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

def pointLineDist(point, line):
	return np.linalg.norm(np.cross(line[1]-line[0], line[0]-point))/np.linalg.norm(line[1]-line[0])

def metric(particle, history):
	best = []
	indices = None

	if randHist:
		indices = random.sample(range(len(history)-1), min(5, len(history)-1))
		indices.append(len(history)-1)
	else:
		indices = range(max(0, len(history)-5), len(history))

	for i in indices:
		startPoint = history[i][0]
		endPoints = history[i][1]

		dists = []
		for i in range(0, len(endPoints)):
			d = pointLineDist(particle.getPrediction(), [startPoint, endPoints[i]])
			d = np.power(d, 0.25)
			dists.append(d)

		best.append(np.min(dists))
	return np.sum(best)

def main():
	rospy.init_node("particle_based_tracking")

	detector = Detector(visualize=False)
	transformer = Transformer("transformer")
	pf = ParticleFilter(500, HParticle, metric, explorationFactor=0.1, noiseFactor=0.05, averageType="weighted")
	pf.generateParticles()
	viz = PFViz(pf, "/odom", "myViz")

	history = []

	while not rospy.is_shutdown():
		try:
			detector.getImage()
			detector.processImage()
			pixels = detector.centroids[:]

			startPoint, endPoints = transformer.transform(pixels)
			history.append(tuple((startPoint[:-1], np.asarray(endPoints)[:,:-1])))
			T0 = time.time()
			for _ in range(0, numItersPerSample):
				print "Updating particle filter",

				print "\tmeasuring",
				t0 = time.time()
				pf.measureParticles(history)
				t1 = time.time()
				print "dt=%f" % (t1-t0),

				print "\tweighting",
				t0 = time.time()
				pf.calculateWeights()
				t1 = time.time()
				print "dt=%f" % (t1-t0),

				print "\tpredicting",
				t0 = time.time()
				prediction = pf.predict()
				t1 = time.time()
				print "dt=%f" % (t1-t0),

				viz.update(history[-1])

				print "\tresampling",
				t0 = time.time()
				pf.resample()
				t1 = time.time()
				print "dt=%f" % (t1-t0),

				pf.update(None)

				print
			T1 = time.time()
			print "Total particle filter update time %f" % (T1-T0)
		except KeyboardInterrupt:
			break

if __name__ == "__main__":
	main()