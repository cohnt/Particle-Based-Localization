import rospy
import roslib
import tf
import math
import time
import sys
import numpy as np

from nav_msgs.msg import Odometry as OdometryMsg
from geometry_msgs.msg import PointStamped as PointStampedMsg
from geometry_msgs.msg import Point as PointMsg
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from tf2_msgs.msg import TFMessage as TFMsg

from cv2 import projectPoints

class FakeDetector():
	def __init__(self, name, bagHandlePoint=[0.95, -0.5, 1.1, 1]):
		self.name = name
		self.bagHandlePoint = bagHandlePoint
		self.cameraFOV = np.array([54.0, 45.0]) * math.pi / 180.0
		self.imageDims = np.array([320.0, 240.0])

		self.tfListener = tf.TransformListener()

		self.pub = rospy.Publisher(name, PointCloud, queue_size=10)

	def unit_vector(self, vector):
		return vector / np.linalg.norm(vector)

	def angle_between(self, v1, v2):
		v1_u = self.unit_vector(v1)
		v2_u = self.unit_vector(v2)
		return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

	def transform3dToImg(self, points):
		pixels = []
		waiting = False
		while True:
			try:
				stamp = self.tfListener.getLatestCommonTime("/head_camera_rgb_optical_frame", "/odom")
				pos, quat = self.tfListener.lookupTransform("/head_camera_rgb_optical_frame", "/odom", stamp)
				matTransform = self.tfListener.fromTranslationRotation(pos, quat)

				print "Got transform!"
				
				for point in points:
					# print "Point", point,

					point = np.matmul(matTransform, point)

					horiz = self.angle_between([point[0], 0, point[2]], [0, 0, 1])
					vert  = self.angle_between([0, point[1], point[2]], [0, 0, 1])

					if point[0] < 0:
						horiz = horiz * -1
					if point[1] < 0:
						vert = vert * -1

					pixel = [0, 0]
					pixel[0] = ((horiz / self.cameraFOV[0]) * self.imageDims[0])
					pixel[1] = ((vert / self.cameraFOV[1]) * self.imageDims[1])

					# print " *becomes* ", pixel
					pixels.append(pixel)
				break
			except:
				if not waiting:
					waiting = True
					sys.stdout.write("Waiting for transform")
				else:
					sys.stdout.write('.')
				sys.stdout.flush()
				time.sleep(0.1)
		return pixels

	def fakeDetect(self):
		pc = PointCloud()
		
		pc.header.stamp = rospy.Time.now()
		pc.header.frame_id = 'odom'
		pc.channels = []
		pc.points.append(Point32(self.bagHandlePoint[0], self.bagHandlePoint[1], self.bagHandlePoint[2]))

		self.pub.publish(pc)

		time.sleep(0.5)
		return self.transform3dToImg([self.bagHandlePoint])

if __name__ == "__main__":
	fd = FakeDetector("fake_detector")
	while not rospy.is_shutdown():
		try:
			pixels = fd.fakeDetect()
		except KeyboardInterrupt:
			break