# Primarily based on grab_bag_behavior.py, from behavior_manager on branch lfd

import rospy
import actionlib
import tf
from math import radians, atan2, cos, sin
import numpy as np
from copy import deepcopy
import time
import sys

from grab_bag.msg import GrabBagAction, GrabBagGoal
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def write(*args, **kwargs):
	sys.stdout.write(*args, **kwargs)

def flush(*args, **kwargs):
	sys.stdout.flush(*args, **kwargs)

class Grabber:
	def __init__(self, grab_frame="base_link"):
		self.grab_frame = grab_frame
		self.tfListener = tf.TransformListener()

	def grab(self, h1, h2, frame):
		stamp = self.tfListener.getLatestCommonTime(frame, self.grab_frame)
		pos, quat = self.tfListener.lookupTransform(self.grab_frame, frame, stamp)
		matTransform = self.tfListener.fromTranslationRotation(pos, quat)

		h1Transformed = np.matmul(matTransform, np.append(h1, 1))
		h2Transformed = np.matmul(matTransform, np.append(h2, 1))

		front = None
		back = None
		x1 = h1Transformed[0]
		x2 = h2Transformed[0]
		if x1 > x2:
			back = deepcopy(h1Transformed)
			front = deepcopy(h2Transformed)
		else:
			front = deepcopy(h1Transformed)
			back = deepcopy(h2Transformed)

		frontPose = Pose()
		backPose = Pose()

		frontPose.position.x = front[0]
		frontPose.position.y = front[1]
		frontPose.position.z = front[2]

		backPose.position.x = back[0]
		backPose.position.y = back[1]
		backPose.position.z = back[2]

		x = 90
		y = atan2(backPose.position.y-frontPose.position.y, backPose.position.x - frontPose.position.x)
		z = 0
		quaternion = tf.transformations.quaternion_from_euler(radians(x), y, 0, 'rxyz')

		frontPose.orientation.x = quaternion[0]
		frontPose.orientation.y = quaternion[1]
		frontPose.orientation.z = quaternion[2]
		frontPose.orientation.w = quaternion[3]

		backPose.orientation.x = quaternion[0]
		backPose.orientation.y = quaternion[1]
		backPose.orientation.z = quaternion[2]
		backPose.orientation.w = quaternion[3]


		backPose.position.x = backPose.position.x - 0.06 * cos(y)
		backPose.position.y = backPose.position.y - 0.06 * sin(y)
		backPose.position.z = ((backPose.position.z + frontPose.position.z) / 2.0) + 0.06

		frontPose.position.x = frontPose.position.x - 0.25 * cos(y)
		frontPose.position.y = frontPose.position.y - 0.25 * sin(y)
		frontPose.position.z = ((backPose.position.z + frontPose.position.z) / 2.0) + 0.06

		client = actionlib.SimpleActionClient("grab_bag", GrabBagAction)
		client.wait_for_server()

		goal = GrabBagGoal()
		goal.pre_grasp_pose = frontPose
		goal.grasp_pose = backPose
		goal.cartesian_max_try = 3
		client.sendGoal(goal)

		client.wait_for_result()

		print client.get_result()