from detector import Detector
from particle_filter import ParticleFilter, Particle
from pf_viz import PFViz
from transformer import Transformer

import rospy
import numpy as np
import random
import time

environmentBounds = [[-2, 2], [-2, 2], [0, 2]]
expectedHandleDist = 0.12

numItersPerSample = 10 # Number of times to iterate the particle filter per scan received
numHist = 3           # Number of scans to use in weighting particles
randHist = True       # If true, select random scans from the history, as opposed to the most recent ones

class HParticle(Particle):
	def __init__(self, pos=[0, 0, 0]):
		self.pos = [0, 0, 0]
		self.error = 0
		self.weight = 0

	def randomize(self):
		# For dimension i, and range [x,y), we let self.pos[i]~Unif[x, y)
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
		# Add noise n~Unif[-noiseFactor, noiseFactor) to each dimension
		self.pos[0] = self.pos[0] + (random.random()*2*noiseFactor - noiseFactor)
		self.pos[1] = self.pos[1] + (random.random()*2*noiseFactor - noiseFactor)
		self.pos[2] = self.pos[2] + (random.random()*2*noiseFactor - noiseFactor)
		# TODO: Add noise of magnitude n~Unif[0,noiseFactor) and uniform random direction

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

def squaredNorm(point):
	return np.sum(np.multiply(point, point))

def pointLineDist(point, line):
	# Returns the distance between a point [x,y,z] and a line [[x1,y1,z1],[x2,y2,z2]]
	return squaredNorm(np.cross(line[1]-line[0], line[0]-point))/squaredNorm(line[1]-line[0])

def angularDist(point1, point2):
	num = np.dot(point1, point2)
	denom = np.sqrt(squaredNorm(point1)*squaredNorm(point2))
	return np.arccos(num/denom)

def metric(particle, history):
	best = []
	indices = None

	# If randHist is true, select n-1 random entries from the history of scans, as well as
	# the most recent one. Otherwise, select the n most recent scan. Note that if n=1,
	# then this doesn't matter, as it will always use the most recent scan.
	if randHist:
		indices = random.sample(range(len(history)-1), min(numHist, len(history)-1))
		indices.append(len(history)-1)
	else:
		indices = range(max(0, len(history)-5), len(history))

	# For each scan selected, we need to find the distance from the prediction to the
	# ray which is closest to the particle. Note that the start and end points of the
	# lines are stored in separate lists (history[i][0] and history[i][1])
	for i in indices:
		startPoint = history[i][0]
		endPoints = history[i][1]

		dists = []
		for i in range(0, len(endPoints)):
			# d = pointLineDist(particle.getPrediction(), [startPoint, endPoints[i]])
			d = angularDist(particle.getPrediction()-startPoint, endPoints[i]-startPoint)
			dists.append(d)

		best.append(np.min(dists))

	# TODO: Figure out why this distance penalty gives a tighter particle cluster. I would
	#       expect that higher powers would have this effect, but they end up giving wider
	#       distributions and fractional powers (i.e. radicals, i.e. nth roots) give a
	#       tighter distribution
	return np.sum(np.power(best, 0.125))

def metric2(particle, (history, otherPrediction)):
	best = []
	indices = None

	# If randHist is true, select n-1 random entries from the history of scans, as well as
	# the most recent one. Otherwise, select the n most recent scan. Note that if n=1,
	# then this doesn't matter, as it will always use the most recent scan.
	if randHist:
		indices = random.sample(range(len(history)-1), min(numHist, len(history)-1))
		indices.append(len(history)-1)
	else:
		indices = range(max(0, len(history)-5), len(history))

	# For each scan selected, we need to find the distance from the prediction to the
	# ray which is closest to the particle. Note that the start and end points of the
	# lines are stored in separate lists (history[i][0] and history[i][1])
	for i in indices:
		startPoint = history[i][0]
		endPoints = history[i][1]

		dists = []
		for i in range(0, len(endPoints)):
			# d = pointLineDist(particle.getPrediction(), [startPoint, endPoints[i]])
			d = angularDist(particle.getPrediction()-startPoint, endPoints[i]-startPoint)
			dists.append(d)

		best.append(np.min(dists))

	# TODO: Figure out why this distance penalty gives a tighter particle cluster. I would
	#       expect that higher powers would have this effect, but they end up giving wider
	#       distributions and fractional powers (i.e. radicals, i.e. nth roots) give a
	#       tighter distribution

	handleDist = np.sqrt(squaredNorm(np.asarray(particle.getPrediction()) - np.asarray(otherPrediction)))
	handlePenalty = 5.0 * (1.0 - np.exp(-1.0 * (handleDist - expectedHandleDist)**2)) + 1.0

	return (np.sum(np.power(best, 0.125)) + 1) * handlePenalty

def main():
	rospy.init_node("particle_based_tracking")

	# See respective files for details on class constructors
	detector = Detector(visualize=False)
	transformer = Transformer("transformer")
	pf = ParticleFilter(500, HParticle, metric, explorationFactor=0.1, noiseFactor=0.1, averageType="weighted")
	pf2 = ParticleFilter(500, HParticle, metric2, explorationFactor=0.1, noiseFactor=0.1, averageType="weighted")
	
	viz = PFViz(pf, "/odom", "myViz", markerColor=[0, 0, 0, 1])
	viz2 = PFViz(pf2, "/odom", "myViz2", markerColor=[1, 1, 1, 1])

	pf.generateParticles()
	pf2.generateParticles()
	history = []

	raw_input("\nProgram ready. Press [Enter] to start. Then [Ctrl] + [c] to stop.")

	while not rospy.is_shutdown():
		try:
			print "\nWaiting for the next image."
			detector.getImage()
			detector.processImage()
			pixels = detector.centroids[:]
			stamp = detector.imageStamp

			startPoint, endPoints, success = transformer.transform(pixels, stamp) # Convert images pixels to 3D points in the /odom frame
			if not success:
				continue
			
			history.append(tuple((startPoint[:-1], np.asarray(endPoints)[:,:-1])))

			if len(history) < numHist:
				continue

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

				print "Updating particle filter 2",

				print "\tmeasuring",
				t0 = time.time()
				pf2.measureParticles((history, pf.predict()))
				t1 = time.time()
				print "dt=%f" % (t1-t0),

				print "\tweighting",
				t0 = time.time()
				pf2.calculateWeights()
				t1 = time.time()
				print "dt=%f" % (t1-t0),

				print "\tpredicting",
				t0 = time.time()
				prediction = pf2.predict()
				t1 = time.time()
				print "dt=%f" % (t1-t0),

				viz2.update(history[-1])

				print "\tresampling",
				t0 = time.time()
				pf2.resample()
				t1 = time.time()
				print "dt=%f" % (t1-t0),

				pf2.update(None)

				print
			T1 = time.time()
			print "Total particle filter update time %f" % (T1-T0)
		except KeyboardInterrupt:
			break

	print "Final estimate for handle 1 position: [%f,%f,%f]" % tuple(pf.predict())
	print "Final estimate for handle 2 position: [%f,%f,%f]" % tuple(pf2.predict())

if __name__ == "__main__":
	main()