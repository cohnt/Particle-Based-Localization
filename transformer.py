import rospy
import roslib
import tf
import math
import numpy as np

from nav_msgs.msg import Odometry as OdometryMsg
from geometry_msgs.msg import PointStamped as PointStampedMsg
from geometry_msgs.msg import Point as PointMsg
from tf2_msgs.msg import TFMessage as TFMsg

class Transformer:
	def __init__(self, name, FOV=np.array([54.0, 45.0]) * math.pi / 180.0):
		self.name = name
		self.FOV = FOV
		self.depth = 1.0

		rospy.init_node("%s_tf" % self.name)
		self.tfListener = tf.TransformListener()

	def transform(self, points):
		horizFOV = self.FOV[0]
		vertFOV = self.FOV[1]
		depth = 1.0

		horizRadsPerPixel = horizFOV / 640.0
		vertRadsPerPixel = vertFOV / 480.0

		angles = np.array([horizRadsPerPixel, vertRadsPerPixel])

		newPoints = []

		while True:
			try:
				stamp = self.tfListener.getLatestCommonTime("/head_camera_rgb_optical_frame", "/odom")
				pos, quat = self.tfListener.lookupTransform("/odom", "/head_camera_rgb_optical_frame", stamp)
				matTransform = self.tfListener.fromTranslationRotation(pos, quat)

				startPoint = np.array([0, 0, 0, 1])
				startPoint = np.matmul(matTransform, startPoint)

				for point in points:
					endPixel = np.array(point)

					endAngs = angles * endPixel

					endRay = [math.sin(endAngs[0]), -43]

					h1 = math.sqrt(1 + endRay[0]**2)

					endRay[1] = h1 * math.sin(endAngs[1])

					endPoint = np.array([endRay[0], endRay[1], 1, 1])
					endPoint = np.matmul(matTransform, endPoint)

					newPoints.append(endPoint.copy())
				break
			except:
				print "Waiting for transform..."

		return newPoints