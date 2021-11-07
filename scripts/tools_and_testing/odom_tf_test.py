import rospy
import tf

from nav_msgs.msg import Odometry as OdometryMsg
from tf2_msgs.msg import TFMessage as TFMsg

def odomCallback(data, tfListener):
	pos = data.pose.pose.position
	rospy.loginfo("Position: [%f,%f,%f]" % (pos.x, pos.y, pos.z))
	(trans, rot) = tfListener.lookupTransform("/odom", "/head_camera_rgb_frame", rospy.Time(0))
	rospy.loginfo(trans)
	rospy.loginfo(rot)

def main():
	rospy.init_node("odom_tf_test")
	tfListener = tf.TransformListener()
	rospy.Subscriber("/odom", OdometryMsg, odomCallback, tfListener)

	while not rospy.is_shutdown():
		pass

if __name__ == "__main__":
	main()