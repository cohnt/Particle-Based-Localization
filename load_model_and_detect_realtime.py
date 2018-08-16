#!/usr/bin/env python2

import rospy
import os
import pickle
from sensor_msgs.msg import Image
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

import time

# np.set_printoptions(threshold='nan')

cell_size = (8, 8) # x, y
window_size = (16, 16) # x, y

plt.ion()
ready = True

fig, ax = plt.subplots()
ax.axis('off')
imgObj = 0

isHandleData = 0
notHandleData = 0

fd = 0

svc = 0
learned = False
imageMode = False

clusterMaxDistance = 3
clusterMinSamples = 5

db = 0

bridge = CvBridge()

lastmsg = 0

def image_callback(msg):
	# global lastmsg, ready
	# print "Received an image."
	# if ready:
	# 	lastmsg = msg
	# 	process_image()

	global lastmsg, ready
	print "Received an image."
	if ready:
		lastmsg = msg
		try:
			process_image()
		except:
			ready = True
	else:
		return

def process_image():
	# myTime = rospy.get_time()
	# yourTime = msg.header.stamp.to_sec()
	# print "Current time: %s" % myTime
	# print "Message time: %s" % yourTime
	# print "Elapsed: %s" % (myTime-yourTime)

	global lastmsg, im1, im2, new_img, imgObj, ready, fd, hog_image, hog_image_rescaled

	ready = False

	print "Processing an image"
	msg = lastmsg
	try:
		# Convert your ROS Image message to OpenCV2
		# print "Converting...",
		img = bridge.imgmsg_to_cv2(msg, "bgr8")
		# print "Done!"
	except CvBridgeError, e:
		print(e)
		ready = True
	else:
		# print "Processing... ",
		startTime = time.time()
		fd, hog_image = hog(rgb2gray(img), orientations=8, pixels_per_cell=cell_size, cells_per_block=(1, 1), visualise=True, feature_vector=False, block_norm='L2')
		hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10))
		endTime = time.time()
		# print "Done! ",

		# print "Time: %s" % (endTime-startTime)

		# print "Updating display... ",
		startTime = time.time()
		if imgObj == 0:
			imgObj = ax.imshow(hog_image_rescaled, cmap=plt.cm.gray)
		elif not imageMode:
			imgObj.set_data(hog_image_rescaled)
		else:
			imgObj.set_data(img)
		endTime = time.time();
		# print "Done! ",
		# print "Time: %s" % (endTime-startTime)

		if learned:
			predictImage()

		ready = True

class WindowIndicator(object):
	def __init__(self, ax):
		self.ax = ax
		self.vl1 = ax.axvline(color='red', alpha=0.75)
		self.vl2 = ax.axvline(color='red', alpha=0.75)
		self.hl1 = ax.axhline(color='red', alpha=0.75)
		self.hl2 = ax.axhline(color='red', alpha=0.75)
		self.x = 0
		self.y = 0
	
	def mouse_move(self, event):
		if not event.inaxes: return
		x, y = event.xdata, event.ydata
		self.vl1.set_xdata(x-(x % cell_size[0]))
		self.vl2.set_xdata(x+(cell_size[0]*window_size[0])-(x % cell_size[0]))
		self.hl1.set_ydata(y-(y % cell_size[1])-1)
		self.hl2.set_ydata(y+(cell_size[1]*window_size[1])-(y % cell_size[1])-1)
		self.ax.figure.canvas.draw_idle()

def clickClassify(cell_id):
	global fd, svc

	temp = fd[cell_id[0]:cell_id[0]+window_size[1],cell_id[1]:cell_id[1]+window_size[0],:,:,:]
	print np.shape(temp)
	temp = np.reshape(temp, (-1))
	temp = np.reshape(temp, (1, -1))

	print "Predicted:"
	print svc.predict(temp)

def onclick(event):
	global fd, isHandleData, notHandleData, svc, cell_size, window_size, learned
	if ax.in_axes(event):
		# Transform the event from display to axes coordinates
		ax_pos = ax.transAxes.inverted().transform((event.x, event.y))
		exact_pos = ax_pos * (640, 480)
		exact_pos[1] = 480 - exact_pos[1]
		exact_pos = exact_pos - (0.5, 0.5)
		print(exact_pos)
		cell_id = np.copy(exact_pos)
		cell_id[0] = np.floor(exact_pos[1] / cell_size[1])
		cell_id[1] = np.floor(exact_pos[0] / cell_size[0])
		cell_id = cell_id.astype(int)
		print(cell_id)
		if learned:
			clickClassify(cell_id)
		if event.button == 1:
			print "NOT HANDLE"
			if np.shape(notHandleData) == ():
				notHandleData = [fd[cell_id[0]:cell_id[0]+window_size[1],cell_id[1]:cell_id[1]+window_size[0],:,:,:]]
			else:
				notHandleData = np.append(notHandleData, [fd[cell_id[0]:cell_id[0]+window_size[1],cell_id[1]:cell_id[1]+window_size[0],:,:,:]], axis=0)
		elif event.button == 3:
			print "HANDLE"
			if np.shape(isHandleData) == ():
				isHandleData = [fd[cell_id[0]:cell_id[0]+window_size[1],cell_id[1]:cell_id[1]+window_size[0],:,:,:]]
			else:
				isHandleData = np.append(isHandleData, [fd[cell_id[0]:cell_id[0]+window_size[1],cell_id[1]:cell_id[1]+window_size[0],:,:,:]], axis=0)
	else:
		print("Outside of figure!")

def learn():
	global isHandleData, notHandleData, svc, learned
	svc = LinearSVC(random_state=0)
	positives = np.reshape(isHandleData, (np.shape(isHandleData)[0], -1))
	negatives = np.reshape(notHandleData, (np.shape(notHandleData)[0], -1))

	positivesBool = np.ones(np.shape(positives)[0])
	negativesBool = np.zeros(np.shape(negatives)[0])

	X = np.concatenate((positives, negatives))
	y = np.concatenate((positivesBool, negativesBool))

	svc.fit(X, y)

	print "Done learning!"
	learned = True

def onkeypress(event):
	global isHandleData, notHandleData, svc, learned, imNum, imageMode
	print "Key pressed: ",
	print event.key
	if event.key == "l": # Learn
		learn()
	elif event.key == "i": # Switch mode from HOG to image
		imageMode = not imageMode

def classify(cell_id):
	global fd, svc

	temp = fd[cell_id[0]:cell_id[0]+window_size[1],cell_id[1]:cell_id[1]+window_size[0],:,:,:]
	# print np.shape(temp)
	temp = np.reshape(temp, (-1))
	temp = np.reshape(temp, (1, -1))

	return svc.predict(temp)

def predictImage():
	global fd, svc, ax, db, window_size, cell_size

	[p.remove() for p in reversed(ax.patches)]

	guesses = []

	for i in range(0, np.shape(fd)[0]-window_size[1]):
		for j in range(0, np.shape(fd)[1]-window_size[0]):
			cell_id = (i, j)
			# print cell_id,
			if classify(cell_id):
				# print "Yes!"
				guesses.append(cell_id)
				ax.add_patch(
					patches.Rectangle(
						(j*cell_size[0]+(0.5*(window_size[0]*cell_size[0])), i*cell_size[1]+(0.5*(window_size[1]*cell_size[1]))),
						2,
						2,
						fill=False,
						edgecolor="blue"
					)
				)
			# else:
				# print ""
	guesses = np.array(guesses)
	db.fit(guesses)
	labels = db.labels_
	unique_labels = set(labels)
	print(db.labels_)
	numLabels = len(unique_labels)
	print "Found %s clusters" % numLabels
	for k in unique_labels:
		if k == -1:
			continue
		# print labels
		# print k
		class_member_mask = (labels == k)
		# print class_member_mask
		# print np.shape(class_member_mask)
		# print guesses
		# print np.shape(guesses)
		xy = np.matrix(guesses[class_member_mask])
		print(xy)
		centroid = np.mean(xy, axis=0)
		print(centroid)
		ax.add_patch(
			patches.Rectangle(
				(centroid[0, 1]*cell_size[0], centroid[0, 0]*cell_size[1]),
				window_size[0]*cell_size[0],
				window_size[1]*cell_size[1],
				fill=False,
				edgecolor="blue"
			)
		)

def openModel():
	global isHandleData, notHandleData, learned
	if not os.path.isfile("model.pickle"):
		raise Exception("File missing")
	with open("model.pickle", "rb") as f:
		temp = pickle.load(f)
		isHandleData = temp[0]
		notHandleData = temp[1]
		print np.shape(isHandleData)
		print np.shape(notHandleData)

def main():
	global db

	openModel()
	learn()

	db = DBSCAN(eps=clusterMaxDistance, n_jobs=-1, min_samples=clusterMinSamples)

	cursor = WindowIndicator(ax)
	cid1 = plt.connect('motion_notify_event', cursor.mouse_move)
	cid2 = plt.connect('button_press_event', onclick)
	cid3 = plt.connect('key_press_event', onkeypress)

	mng = plt.get_current_fig_manager()
	mng.resize(*mng.window.maxsize())

	plt.rcParams["keymap.yscale"] = ""

	rospy.init_node('image_listener')
	image_topic = "/head_camera/rgb/image_raw"
	# rospy.Subscriber(image_topic, Image, image_callback)

	while not rospy.is_shutdown():
		try:
			fig.canvas.draw()
			msg = rospy.wait_for_message(image_topic, Image)
			image_callback(msg)
		except KeyboardInterrupt:
			break

if __name__ == '__main__':
	main()