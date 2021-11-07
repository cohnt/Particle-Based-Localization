import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

def talker():
	pub = rospy.Publisher('pc_test', PointCloud, queue_size=10)
	rospy.init_node('publish_pc')
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		pc = PointCloud()
		
		# pc.header.stamp = rospy.Time.now()
		# pc.header.frame_id = 'map'

		# numPoints = 3
		# pc.channels = []
		# for i in range(numPoints):
		# 	pc.points.append(Point32(i, i, 0))

		pc.header.stamp = rospy.Time.now()
		pc.header.frame_id = 'odom'
		pc.channels = []
		pc.points.append(Point32(0.95, -0.5, 1.1))

		pub.publish(pc)
		print pc
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass