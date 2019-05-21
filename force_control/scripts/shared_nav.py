#!/usr/bin/python
import rospy
import numpy as np
import tf
#import scipy.signal as sg
from geometry_msgs.msg import Twist, Wrench, Pose, PoseStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe


class SharedNav(object):
	def __init__(self):
		'''Parameters Inicialization '''
		self.rospy = rospy
		self.rospy.init_node("SharedNavPub", anonymous = True)
		self.pose_user_topic = self.rospy.get_param("pose_user_topic","/pose_force")
		self.odom_topic = self.rospy.get_param("odom_topic","/RosAria/pose")
		self.plan_poses_topic = self.rospy.get_param("plan_poses_topic","/move_base/TebLocalPlannerROS/local_plan")
		self.nav_goal_topic = self.rospy.get_param("nav_goal_topic","/move_base_simple/goal")
		self.tf_topic = self.rospy.get_param("tf_topic","/tf")
		self.advertise_nav_topic = self.rospy.get_param("advertise_nav_topic","shared_nav_topic")
		self.markers_topic = self.rospy.get_param("/markers_topic", "/markers")
		self.tf_listener = tf.TransformListener()
		'''Internal Var'''
		self.change1 = 	self.change2 = self.change3 = self.change4 = self.change5 = False
		self.pose_user = Odometry()
		self.pose_walker = Odometry()
		self.path = Path()
		self.goal = PoseStamped()
		self.shared_nav_mode = False
		self.win_width = 0.8
		self.max_angle = 90*np.pi/180
		'''Subscribers'''
		self.sub_pose_user = self.rospy.Subscriber(self.pose_user_topic, Odometry, self.callback_pose_user)
		self.sub_odom = self.rospy.Subscriber(self.odom_topic, Odometry, self.callback_odom)
		self.sub_plan_poses = self.rospy.Subscriber(self.plan_poses_topic, Path, self.callback_path)
		self.sub_nav_goal = self.rospy.Subscriber(self.nav_goal_topic, PoseStamped, self.callback_nav_goal)
		self.sub_tf = self.rospy.Subscriber(self.tf_topic, TFMessage, self.callback_tf)
		#self.tf_buffer = tf2_ros.Buffer()
		#self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		'''Publishers'''
		self.pub_advertise_nav = self.rospy.Publisher(self.advertise_nav_topic, Bool, queue_size = 10)
		self.pub_markers = self.rospy.Publisher(self.markers_topic, MarkerArray, queue_size=10)
		'''Node Configuration'''
		self.rospy.init_node("SharedNavPub", anonymous = True)
		self.rate = self.rospy.Rate(20)
		self.main()

	def callback_pose_user(self,msg):
		self.pose_user = msg
		theta_aux = efq([0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
		#self.thetaUser = np.arccos(theta_aux)*2
		self.thetaUser = theta_aux[2]
		self.change1 = True
		return

	def callback_odom(self,msg):
		self.pose_walker = msg
		theta_aux = efq([0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
		#self.thetaWalker = np.arccos(theta_aux)*2
		self.thetaWalker = theta_aux[2]
		self.change2 = True
		return

	def callback_path(self, msg):
		self.path = msg
		try:
			self.nextPose = self.path.poses[5].pose
		except:
			self.nextPose = self.path.poses[0].pose
		theta_aux = efq([0, 0, self.nextPose.orientation.z, self.nextPose.orientation.w])
		self.thetaNav = theta_aux[2]
		self.change3 = True
		return

	def callback_nav_goal(self, msg):
		self.goal = msg
		self.change4 = True
		return

	def callback_tf(self, msg):
		if msg.transforms[0].header.frame_id == "map" and msg.transforms[0].child_frame_id == "odom":
			qz = msg.transforms[0].transform.rotation.z
			qw = msg.transforms[0].transform.rotation.w
			self.rotMapOdom = efq([0, 0, qz, qw])[2]
			self.tx = msg.transforms[0].transform.translation.x
			self.ty = msg.transforms[0].transform.translation.y
			self.change5 =  True
		return

	def publish_markers(self, thetaWalker, thetaDiff, thetaA, thetaB):
		markers = []
		marker = Marker()
		marker.header.seq = 1
		marker.header.stamp = self.rospy.get_rostime()
		marker.header.frame_id = "map"
		marker.id = 1
		marker.type = 0
		marker.action = 0
		#translation, rotation = self.tf_listener.lookupTransform("/map", "/odom")
		#matrix = self.tf_listener.fromTranslationRotation(translation, rotation)
		theta = self.rotMapOdom
		rotation =  np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
		translation = np.array([[self.tx], [self.ty]])
		pos = np.array([[self.pose_walker.pose.pose.position.x], [self.pose_walker.pose.pose.position.y]])
		new_pos = rotation.dot(pos) + translation
		marker.pose.position.x = new_pos[0]
		marker.pose.position.y = new_pos[1]
		quat = qfe(0, 0, thetaDiff-thetaA)
		marker.pose.orientation.x = quat[0]
		marker.pose.orientation.y = quat[1]
		marker.pose.orientation.z = quat[2]
		marker.pose.orientation.w = quat[3]
		marker.scale.x = 1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1
		marker.color.r = 0
		marker.color.g = 1
		marker.color.b = 0
		markers.append(marker)
		marker = Marker()
		marker.header.seq = 2
		marker.header.stamp = self.rospy.get_rostime()
		marker.header.frame_id = "map"
		marker.id = 2
		marker.type = 0
		marker.action = 0
		pos = np.array([[self.pose_walker.pose.pose.position.x], [self.pose_walker.pose.pose.position.y]])
		new_pos = rotation.dot(pos) + translation
		marker.pose.position.x = new_pos[0]
		marker.pose.position.y = new_pos[1]
		quat = qfe(0, 0, thetaDiff+thetaB)
		marker.pose.orientation.x = quat[0]
		marker.pose.orientation.y = quat[1]
		marker.pose.orientation.z = quat[2]
		marker.pose.orientation.w = quat[3]
		marker.scale.x = 1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1
		marker.color.r = 0
		marker.color.g = 0
		marker.color.b = 1
		markers.append(marker)
		marker = Marker()
		marker.header.seq = 3
		marker.header.stamp = self.rospy.get_rostime()
		marker.header.frame_id = "map"
		marker.id = 3
		marker.type = 0
		marker.action = 0
		pos = np.array([[self.pose_walker.pose.pose.position.x], [self.pose_walker.pose.pose.position.y]])
		new_pos = rotation.dot(pos) + translation
		marker.pose.position.x = new_pos[0]
		marker.pose.position.y = new_pos[1]
		quat = qfe(0, 0, thetaDiff)
		marker.pose.orientation.x = quat[0]
		marker.pose.orientation.y = quat[1]
		marker.pose.orientation.z = quat[2]
		marker.pose.orientation.w = quat[3]
		marker.scale.x = 1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1
		marker.color.r = 0
		marker.color.g = 1
		marker.color.b = 1
		markers.append(marker)
		self.pub_markers.publish(markers)
		return

	def main(self):
		rotMapOdom = int(input("Ingrese rotacion entre map y odom"))*np.pi/180
		while not(self.rospy.is_shutdown()):
			if self.change1 and self.change2 and self.change3 and self.change4 and self.change5:
				nextPoseX, nextPoseY  = self.nextPose.position.x, self.nextPose.position.y
				userX, userY = self.pose_user.pose.pose.position.x, self.pose_user.pose.pose.position.y
				distanceToNextPose = np.sqrt((nextPoseX-userX)**2+(nextPoseY-userY)**2)
				thetaWindow = np.arctan(0.25/distanceToNextPose)
				#thetaNav = np.arccos(self.nextPose.orientation.w)*2
				#thetaWalker = np.arccos(self.pose_nav.pose.pose.orientation.w)*2 + rotMapOdom
				#thetaUser = np.arccos(self.pose_user.pose.pose.orientation.w)*2 + rotMapOdom
				thetaWalker = np.mod(self.thetaWalker + self.rotMapOdom, 2*np.pi)
				thetaUser = np.mod(self.thetaUser + self.rotMapOdom, 2*np.pi)
				thetaNav = np.mod(self.thetaNav + self.rotMapOdom, 2*np.pi)
				thetaDiff = np.arctan((nextPoseY-userY)/(nextPoseX-userX)) + self.rotMapOdom
				La = (((-self.win_width)/(2.0*self.max_angle))*(thetaNav - thetaWalker)) + (self.win_width/2.0)
				Lb = (((self.win_width)/(2.0*self.max_angle))*(thetaNav - thetaWalker)) + (self.win_width/2.0)
				thetaA = np.arctan(La/distanceToNextPose)
				thetaB = np.arctan(Lb/distanceToNextPose)
				#print(La, Lb, thetaA, thetaB, thetaDiff)
				#print("--------------")
				self.publish_markers(thetaWalker, thetaDiff, thetaA, thetaB)
				print("thetaWalker",thetaWalker,"thetaNav",thetaNav,"thetaWindow",thetaWindow, "thetaUser",thetaUser, "thetaDiff", thetaDiff)
				#print((thetaDiff-thetaA), " <= ", thetaUser, " <= ", (thetaDiff+thetaB))
				if (thetaDiff-thetaA) <= thetaUser <= (thetaDiff+thetaB):
					print("OK")
				else:
					print("not OK")
				self.change1 = self.cahnge2 = self.change3 = False
			self.rate.sleep()


if __name__ == '__main__':
	print('a')
	try:
		ac = SharedNav()
	except rospy.ROSInterruptException:
		pass
		print('b')
