import rospy
import roslib
import tf
import math

from nav_msgs.msg import Odometry as OdometryMsg
from geometry_msgs.msg import PointStamped as PointStampedMsg
from tf2_msgs.msg import TFMessage as TFMsg
from visualization_msgs.msg import Marker as MarkerMsg
from visualization_msgs.msg import MarkerArray as MarkerArrayMsg

markerArray = MarkerArrayMsg()

def odomCallback(data, tfListener):
	global markerArray
	pos = data.pose.pose.position
	newPoint = PointStampedMsg()
	while True:
		try:
			point = PointStampedMsg()
			point.point.x = 0
			point.point.y = 0
			point.point.z = 1
			point.header.frame_id = "/head_camera_rgb_optical_frame"
			point.header.stamp = tfListener.getLatestCommonTime("/head_camera_rgb_optical_frame", "/odom")

			newPoint = tfListener.transformPoint("/odom", point)
			print "Good!"
			break
		except:
			print "Waiting..."

	marker = MarkerMsg()
	marker.header.frame_id = "/odom"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.05
	marker.scale.y = 0.05
	marker.scale.z = 0.05
	marker.color.a = 1
	marker.color.r = 1
	marker.color.g = 1
	marker.color.b = 1
	marker.pose.position.x = newPoint.point.x
	marker.pose.position.y = newPoint.point.y
	marker.pose.position.z = newPoint.point.z

	markerArray = MarkerArrayMsg()
	markerArray.markers.append(marker)

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