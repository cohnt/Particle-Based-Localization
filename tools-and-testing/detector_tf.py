import rospy
import roslib
import tf
import math
import numpy as np

from nav_msgs.msg import Odometry as OdometryMsg
from geometry_msgs.msg import PointStamped as PointStampedMsg
from geometry_msgs.msg import Point as PointMsg
from tf2_msgs.msg import TFMessage as TFMsg
from visualization_msgs.msg import Marker as MarkerMsg
from visualization_msgs.msg import MarkerArray as MarkerArrayMsg

import sys 
sys.path.append('..')

from detector import *

detector = Detector(visualize=False)

markerArray = MarkerArrayMsg()
markerPublisher = 0

arrowLength = 3

def transform(points, stamp, tfListener):
	global markerArray, markerPublisher

	horizFOV = 54.0 * math.pi / 180.0
	vertFOV = 45.0 * math.pi / 180.0
	depth = 1.0

	horizRadsPerPixel = horizFOV / 640.0
	vertRadsPerPixel = vertFOV / 480.0

	angles = np.array([horizRadsPerPixel, vertRadsPerPixel])

	try:
		tfListener.waitForTransform("/odom", "/head_camera_rgb_optical_frame", stamp, rospy.Duration.from_sec(0.5))
		pos, quat = tfListener.lookupTransform("/odom", "/head_camera_rgb_optical_frame", stamp)
		matTransform = tfListener.fromTranslationRotation(pos, quat)

		startPoint = np.array([0, 0, 0, 1])
		startPoint = np.matmul(matTransform, startPoint)

		for point in points:
			endPixel = np.array(point)

			endAngs = angles * endPixel

			endRay = [math.sin(endAngs[0]), -43]

			h1 = math.sqrt(1 + endRay[0]**2)

			endRay[1] = h1 * math.sin(endAngs[1])

			# endRay = arrowLength * endRay

			endPoint = np.array([endRay[0], endRay[1], 1, 1])
			endPoint = np.matmul(matTransform, endPoint)

			# startPoint = PointStampedMsg()
			# endPoint = PointStampedMsg()
			# while True:
			# 	try:

			# 		point = PointStampedMsg()
			# 		point.point.x = endRay[0]
			# 		point.point.y = endRay[1]
			# 		point.point.z = 1
			# 		point.header.frame_id = "/head_camera_rgb_optical_frame"
			# 		point.header.stamp = stamp

			# 		endPoint = tfListener.transformPoint("/odom", point)

			# 		print "Good!"
			# 		break
			# 	except:
			# 		pass

			marker = MarkerMsg()
			marker.header.frame_id = "/odom"
			marker.type = marker.ARROW
			marker.action = marker.ADD
			marker.points = [PointMsg(), PointMsg()]

			# marker.points[0].x = startPoint.point.x
			# marker.points[0].y = startPoint.point.y
			# marker.points[0].z = startPoint.point.z
			# marker.points[1].x = endPoint.point.x
			# marker.points[1].y = endPoint.point.y
			# marker.points[1].z = endPoint.point.z

			marker.points[0].x = startPoint[0]
			marker.points[0].y = startPoint[1]
			marker.points[0].z = startPoint[2]
			marker.points[1].x = endPoint[0]
			marker.points[1].y = endPoint[1]
			marker.points[1].z = endPoint[2]

			marker.scale.x = 0.005
			marker.scale.y = 0.02
			marker.scale.z = 0.1
			marker.color.a = 1
			marker.color.r = 1
			marker.color.g = 0
			marker.color.b = 0

			markerArray.markers.append(marker)
	except:
		return

	# print markerArray

	id = 0
	for marker in markerArray.markers:
		marker.id = id
		id = id + 1

	markerPublisher.publish(markerArray)

def main():
	global markerArray, markerPublisher
	rospy.init_node("odom_tf_test")
	tfListener = tf.TransformListener()

	markerPublisher = rospy.Publisher("/test_markers", MarkerArrayMsg, queue_size=100)

	while not rospy.is_shutdown():
		plt.pause(0.01)
		try:
			detector.getImage()
			detector.processImage()
			transform(detector.centroids, detector.imageStamp, tfListener)
		except KeyboardInterrupt:
			break

if __name__ == "__main__":
	main()