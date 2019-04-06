import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker as MarkerMsg
from visualization_msgs.msg import MarkerArray as MarkerArrayMsg

class PFViz():
	def __init__(self, pf, frame, name, queue_size=10, markerColor=[1, 1, 1, 1]):
		self.pf = pf
		self.frame = frame
		self.name = name

		self.queue_size = queue_size
		self.markerColor = markerColor

		self.pcPub  = rospy.Publisher("%s_pc" % self.name, PointCloud, queue_size=self.queue_size)
		self.markerPub = rospy.Publisher("%s_marker" % self.name, MarkerArrayMsg, queue_size=self.queue_size)

	def update(self):
		pc = PointCloud()

		pc.header.stamp = rospy.Time.now()
		pc.header.frame_id = self.frame

		pc.channels = [ChannelFloat32()]
		pc.channels[0].name = "weight"

		for particle in self.pf.particles:
			pc.points.append(Point32(particle.pos[0], particle.pos[1], particle.pos[2]))
			pc.channels[0].values.append(particle.getWeight())

		prediction = self.pf.predict()

		r = self.markerColor[0]
		g = self.markerColor[1]
		b = self.markerColor[2]
		a = self.markerColor[3]

		marker = MarkerMsg()
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = self.frame
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.05
		marker.scale.y = 0.05
		marker.scale.z = 0.05
		marker.color.a = a
		marker.color.r = r
		marker.color.g = g
		marker.color.b = b
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