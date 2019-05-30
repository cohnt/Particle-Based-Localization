import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

points = []

def talker():
	global points

	pub = rospy.Publisher('pc_test', PointCloud, queue_size=10)
	rospy.init_node('publish_pc')
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		pc = PointCloud()
		
		pc.header.stamp = rospy.Time.now()
		pc.header.frame_id = 'odom'

		pc.channels = []
		for point in points:
			pc.points.append(Point32(point[0], point[1], point[2]))

		pub.publish(pc)
		print pc

		newPoint = list(map(float, raw_input("Next point x,y,z: ").split(",")))
		points.append(newPoint)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass