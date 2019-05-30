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

from cv2 import projectPoints

import sys 
sys.path.append('..')

markerArray = MarkerArrayMsg()
markerPublisher = 0

arrowLength = 3

bagHandlePoint = [0.95, -0.5, 1.1, 1]

# https://codeyarns.com/tag/primesense/ ???
# https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#projectpoints ???
cameraFOV = np.array([54.0, 45.0]) * math.pi / 180.0
imageDims = np.array([320.0, 240.0])
# focalLength = np.divide(imageDims, np.tan(np.divide(cameraFOV, 2)))
# cameraMatrix = np.matrix([[focalLength[0], 0, imageDims[0]],[0, focalLength[1], imageDims[1]], [0, 0, 1]])

def unit_vector(vector):
	return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
	v1_u = unit_vector(v1)
	v2_u = unit_vector(v2)
	return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def transform3dToImg(points, tfListener):
	pixels = []
	while True:
		try:
			stamp = tfListener.getLatestCommonTime("/head_camera_rgb_optical_frame", "/odom")
			pos, quat = tfListener.lookupTransform("/head_camera_rgb_optical_frame", "/odom", stamp)
			matTransform = tfListener.fromTranslationRotation(pos, quat)
			print "Recieved transform!",
			
			for point in points:
				print point,

				point = np.matmul(matTransform, point)

				horiz = angle_between([point[0], 0, point[2]], [0, 0, 1])
				vert  = angle_between([0, point[1], point[2]], [0, 0, 1])

				if point[0] < 0:
					horiz = horiz * -1
				if point[1] < 0:
					vert = vert * -1

				pixel = [0, 0]
				pixel[0] = ((horiz / cameraFOV[0]) * imageDims[0])
				pixel[1] = ((vert / cameraFOV[1]) * imageDims[1])

				print " *becomes* ", pixel
				pixels.append(pixel)
			break
		except:
			print "Waiting for transform..."
	return pixels

def transformImgTo3d(points, tfListener):
	global markerArray, markerPublisher

	print "And going back, from ", points

	markerArray = MarkerArrayMsg()

	horizFOV = cameraFOV[0]
	vertFOV = cameraFOV[1]
	depth = 1.0

	horizRadsPerPixel = horizFOV / 640.0
	vertRadsPerPixel = vertFOV / 480.0

	angles = np.array([horizRadsPerPixel, vertRadsPerPixel])

	while True:
		try:
			stamp = tfListener.getLatestCommonTime("/head_camera_rgb_optical_frame", "/odom")
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

				endPoint = np.array([endRay[0], endRay[1], 1, 1])
				endPoint = np.matmul(matTransform, endPoint)

				marker = MarkerMsg()
				marker.header.frame_id = "/odom"
				marker.type = marker.ARROW
				marker.action = marker.ADD
				marker.points = [PointMsg(), PointMsg()]

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
			break
		except:
			print "Waiting for transform..."

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
		try:
			pixels = transform3dToImg([bagHandlePoint], tfListener)
			transformImgTo3d(pixels, tfListener)
		except KeyboardInterrupt:
			break

if __name__ == "__main__":
	main()