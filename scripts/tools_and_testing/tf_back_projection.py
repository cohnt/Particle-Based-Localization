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

markerArray = MarkerArrayMsg()

def odomCallback(data, tfListener):
	global markerArray

	horizFOV = 54.0 * math.pi / 180.0
	vertFOV = 45.0 * math.pi / 180.0
	depth = 1.0

	horizRadsPerPixel = horizFOV / 640.0
	vertRadsPerPixel = vertFOV / 480.0

	angles = np.array([horizRadsPerPixel, vertRadsPerPixel])

	endPixel1 = np.array([-320, -240])
	endPixel2 = np.array([320, 240])

	endAngs1 = angles * endPixel1
	endAngs2 = angles * endPixel2

	endRay1 = [math.sin(endAngs1[0]), -43]
	endRay2 = [math.sin(endAngs2[0]), -43]

	h1 = math.sqrt(1 + endRay1[0]**2)
	h2 = math.sqrt(1 + endRay2[0]**2)

	endRay1[1] = h1 * math.sin(endAngs1[1])
	endRay2[1] = h2 * math.sin(endAngs2[1])

	pos = data.pose.pose.position
	startPoint = PointStampedMsg()
	endPoint1 = PointStampedMsg()
	endPoint2 = PointStampedMsg()
	while True:
		try:
			point = PointStampedMsg()
			point.point.x = 0
			point.point.y = 0
			point.point.z = 0
			point.header.frame_id = "/head_camera_rgb_optical_frame"
			point.header.stamp = tfListener.getLatestCommonTime("/head_camera_rgb_optical_frame", "/odom")

			startPoint = tfListener.transformPoint("/odom", point)

			point = PointStampedMsg()
			point.point.x = endRay1[0]
			point.point.y = endRay1[1]
			point.point.z = 1
			point.header.frame_id = "/head_camera_rgb_optical_frame"
			point.header.stamp = tfListener.getLatestCommonTime("/head_camera_rgb_optical_frame", "/odom")

			endPoint1 = tfListener.transformPoint("/odom", point)

			point = PointStampedMsg()
			point.point.x = endRay2[0]
			point.point.y = endRay2[1]
			point.point.z = 1
			point.header.frame_id = "/head_camera_rgb_optical_frame"
			point.header.stamp = tfListener.getLatestCommonTime("/head_camera_rgb_optical_frame", "/odom")

			endPoint2 = tfListener.transformPoint("/odom", point)
			print "Good!"
			break
		except:
			print "Waiting..."

	markerArray = MarkerArrayMsg()

	marker = MarkerMsg()
	marker.header.frame_id = "/odom"
	marker.type = marker.ARROW
	marker.action = marker.ADD
	marker.points = [PointMsg(), PointMsg()]

	marker.points[0].x = startPoint.point.x
	marker.points[0].y = startPoint.point.y
	marker.points[0].z = startPoint.point.z
	marker.points[1].x = endPoint1.point.x
	marker.points[1].y = endPoint1.point.y
	marker.points[1].z = endPoint1.point.z

	marker.scale.x = 0.01
	marker.scale.y = 0.02
	marker.scale.z = 0.1
	marker.color.a = 1
	marker.color.r = 1
	marker.color.g = 0
	marker.color.b = 0

	markerArray.markers.append(marker)

	marker = MarkerMsg()
	marker.header.frame_id = "/odom"
	marker.type = marker.ARROW
	marker.action = marker.ADD
	marker.points = [PointMsg(), PointMsg()]

	marker.points[0].x = startPoint.point.x
	marker.points[0].y = startPoint.point.y
	marker.points[0].z = startPoint.point.z
	marker.points[1].x = endPoint2.point.x
	marker.points[1].y = endPoint2.point.y
	marker.points[1].z = endPoint2.point.z

	marker.scale.x = 0.01
	marker.scale.y = 0.02
	marker.scale.z = 0.1
	marker.color.a = 1
	marker.color.r = 0
	marker.color.g = 0
	marker.color.b = 1

	markerArray.markers.append(marker)

	print markerArray

	id = 0
	for marker in markerArray.markers:
		marker.id = id
		id = id + 1

def main():
	global markerArray
	rospy.init_node("odom_tf_test")
	tfListener = tf.TransformListener()

	markerPublisher = rospy.Publisher("/test_markers", MarkerArrayMsg, queue_size=100)

	rospy.Subscriber("/odom", OdometryMsg, odomCallback, tfListener)

	while not rospy.is_shutdown():
		markerPublisher.publish(markerArray)

if __name__ == "__main__":
	main()