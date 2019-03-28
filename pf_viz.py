import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker as MarkerMsg
from visualization_msgs.msg import MarkerArray as MarkerArrayMsg

class PFViz():
	def __init__(self, pf, frame, name, queue_size=10):
		self.pf = pf
		self.frame = frame
		self.name = name
		self.queue_size = queue_size

		self.pcPub  = rospy.Publisher("%s_pc" % self.name, PointCloud, queue_size=self.queue_size)
		self.markerPub = rospy.Publisher("%s_marker" % self.name, MarkerArrayMsg, queue_size=self.queue_size)

		rospy.init_node(self.name)

	def update(self):
		pc = PointCloud()

		pc.header.stamp = rospy.Time.now()
		pc.header.frame_id = self.frame

		pc.channels = []
		for particle in self.pf.particles:
			pc.points.append(Point32(particle.pos[0], particle.pos[1], particle.pos[2]))

		prediction = self.pf.predict()

		marker = MarkerMsg()
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = self.frame
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.05
		marker.scale.y = 0.05
		marker.scale.z = 0.05
		marker.color.a = 1
		marker.color.r = 1
		marker.color.g = 1
		marker.color.b = 1
		marker.pose.position.x = prediction[0]
		marker.pose.position.y = prediction[1]
		marker.pose.position.z = prediction[2]

		markerArray = MarkerArrayMsg()
		markerArray.markers.append(marker)

		id = 0
		for marker in markerArray.markers:
			marker.id = id
			id = id + 1

		self.pcPub.publish(pc)
		self.markerPub.publish(markerArray)