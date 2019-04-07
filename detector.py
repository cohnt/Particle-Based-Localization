import rospy
import os
import pickle
import time

from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError
import cv2

import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
import matplotlib.patches as patches

from skimage.feature import hog
from skimage.color import rgb2gray
from skimage import data, exposure, io
from skimage.io import imread

from sklearn.svm import LinearSVC
from sklearn.datasets import make_classification
from sklearn.cluster import DBSCAN

import numpy as np

class Detector:
	def __init__(self, imageTopic="/head_camera/rgb/image_raw", visualize=False, hogOrientations=8, hogCellSize=(8, 8), hogNorm="L2",
	             windowSize=(16, 16), modelfileName="model.pickle", dbscanEpsilon=3, dbscanMinCluster=5):
		self.visualize = visualize # Whether or not to run a display window

		self.hogOrientations = hogOrientations
		self.hogCellSize = hogCellSize
		self.hogNorm = hogNorm
		self.windowSize = windowSize

		self.imageTopic = imageTopic
		self.imageMsg = None
		self.image = None
		self.imageStamp = None
		self.hogDescriptor = None
		self.hogImage = None

		self.bridge = CvBridge()

		self.dbscan = DBSCAN(eps=dbscanEpsilon, n_jobs=-1, min_samples=dbscanMinCluster)

		self.positives = []
		self.centroids = []

		self.isHandleData = None
		self.notHandleData = None
		self.svc = None

		if self.visualize:
			plt.ion()

			self.fig, self.ax = plt.subplots()
			self.ax.axis('off')
			self.imgObj = None

			self.imageMode = False

			self.rescaledImage = None

			mng = plt.get_current_fig_manager()
			mng.resize(*mng.window.maxsize())

			plt.connect('key_press_event', self.onkeypress)

		self.openModel(modelfileName)
		self.learn()

	def getImage(self):
		self.imageMsg = rospy.wait_for_message(self.imageTopic, ImageMsg)

	def processImage(self):
		# Convert to OpenCV2 image format
		print "Processing image... ",
		startTime = time.time()
		try:
			self.image = self.bridge.imgmsg_to_cv2(self.imageMsg, "bgr8")
			self.imageStamp = self.imageMsg.header.stamp
		except CvBridgeError, e:
			print(e)
			return

		# Compute the hog descriptor of the image
		self.hogDescriptor, self.hogImage = hog(rgb2gray(self.image),
		                                        orientations=self.hogOrientations,
		                                        pixels_per_cell=self.hogCellSize,
		                                        cells_per_block=(1, 1),
		                                        visualise=True,
		                                        feature_vector=False,
		                                        block_norm=self.hogNorm)

		endTime = time.time()
		print "Done! dt=%s" % (endTime - startTime)

		# Run the sliding window detector
		self.predictImage()

	def classifyCell(self, cell_id):
		# This is really ugly
		temp = self.hogDescriptor[cell_id[0]:cell_id[0]+self.windowSize[1],cell_id[1]:cell_id[1]+self.windowSize[0],:,:,:]
		temp = np.reshape(temp, (-1))
		temp = np.reshape(temp, (1, -1))

		return self.svc.predict(temp)

	def predictImage(self):
		if self.visualize:
			# Clear old visual markers
			[p.remove() for p in reversed(self.ax.patches)]

		print "Predicting image... ",
		self.positives = []
		startTime = time.time()
		# Classify across the entire image with the sliding window detector
		guesses = []
		for i in range(0, np.shape(self.hogDescriptor)[0]-self.windowSize[1]):
			for j in range(0, np.shape(self.hogDescriptor)[1]-self.windowSize[0]):
				cell_id = (i, j)
				if self.classifyCell(cell_id):
					guesses.append(cell_id)
					actualPos = [j*self.hogCellSize[0]+(0.5*(self.windowSize[0]*self.hogCellSize[0])),
								 i*self.hogCellSize[1]+(0.5*(self.windowSize[1]*self.hogCellSize[1]))]
					self.positives.append(actualPos)
					if self.visualize:
						self.ax.add_patch(
							patches.Rectangle(
								(j*self.hogCellSize[0]+(0.5*(self.windowSize[0]*self.hogCellSize[0])),
								 i*self.hogCellSize[1]+(0.5*(self.windowSize[1]*self.hogCellSize[1]))),
								2,
								2,
								fill=False,
								edgecolor="blue"
							)
						)
		guesses = np.array(guesses)

		# Cluster guesses with DBSCAN
		self.centroids = []
		self.dbscan.fit(guesses)
		labels = self.dbscan.labels_
		unique_labels = set(labels)
		numLabels = len(unique_labels)
		for k in unique_labels:
			if k == -1:
				continue
			class_member_mask = (labels == k)
			xy = np.matrix(guesses[class_member_mask])
			centroid = np.mean(xy, axis=0)
			print("Centroid cell: ", centroid)
			actualPos = [centroid[0, 1]*self.hogCellSize[0]+(0.5*(self.windowSize[0]*self.hogCellSize[0])),
						 centroid[0, 0]*self.hogCellSize[1]+(0.5*(self.windowSize[1]*self.hogCellSize[1]))]
			# self.centroids.append(tuple(np.ravel(centroid)))
			print("Centroid pixel: ", actualPos)
			self.centroids.append(tuple(actualPos))
			if self.visualize:
				self.ax.add_patch(
					patches.Rectangle(
						(centroid[0, 1]*self.hogCellSize[0], centroid[0, 0]*self.hogCellSize[1]),
						self.windowSize[0]*self.hogCellSize[0],
						self.windowSize[1]*self.hogCellSize[1],
						fill=False,
						edgecolor="blue"
					)
				)

		endTime = time.time()
		print(self.centroids)
		print "Done! dt=%s" % (endTime - startTime)

	def updateDisplay(self):
		if not self.visualize:
			raise RuntimeError("This detector has visualization disabled!")

		# Rescale hog image for better display
		self.rescaledImage = exposure.rescale_intensity(self.hogImage, in_range=(0, 10))

		# Store the appropriate image into the display object (creating it if necessary)
		if self.imgObj == None and not self.imageMode:
			self.imgObj = self.ax.imshow(self.rescaledImage, cmap=plt.cm.gray)
		elif self.imgObj == None and self.imageMode:
			self.imgObj = self.ax.imshow(self.image)
		elif not self.imageMode:
			self.imgObj.set_data(self.rescaledImage)
		else:
			self.imgObj.set_data(self.image)

		self.fig.canvas.draw()

	def openModel(self, filename):
		if not os.path.isfile(filename):
			raise Exception("File missing")
		with open(filename, "rb") as f:
			temp = pickle.load(f)
			self.isHandleData = temp[0]
			self.notHandleData = temp[1]

	def learn(self):
		self.svc = LinearSVC(random_state=0)
		positives = np.reshape(self.isHandleData, (np.shape(self.isHandleData)[0], -1))
		negatives = np.reshape(self.notHandleData, (np.shape(self.notHandleData)[0], -1))

		positivesBool = np.ones(np.shape(positives)[0])
		negativesBool = np.zeros(np.shape(negatives)[0])

		X = np.concatenate((positives, negatives))
		y = np.concatenate((positivesBool, negativesBool))

		self.svc.fit(X, y)

	def onkeypress(self, event):
		if event.key == "i": # Switch mode from HOG to image (or vice versa)
			self.imageMode = not self.imageMode
			self.updateDisplay()

if __name__ == "__main__":
	rospy.init_node('image_listener')

	detector = Detector(visualize=True)

	while not rospy.is_shutdown():
		plt.pause(0.01)
		try:
			detector.getImage()
			detector.processImage()
			detector.updateDisplay()
		except KeyboardInterrupt:
			break