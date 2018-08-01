#!/usr/bin/python
import rospy
import numpy as np
#import scipy.signal as sg
from geometry_msgs.msg import Twist, Wrench, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool

class SharedNav(object):
	def __init__(self):
		'''Parameters Inicialization '''
		self.rospy = rospy
		self.pose_user_topic = self.rospy.get_param("pose_user_topic","/pose_force")
		self.odom_topic = self.rospy.get_param("odom_topic","/RosAria/pose")
		self.plan_poses_topic = self.rospy.get_param("plan_poses_topic","/move_base/TebLocalPlannerROS/local_plan")
		self.nav_goal_topic = self.rospy.get_param("nav_goal_topic","/move_base_simple/goal")
		self.advertise_nav_topic = self.rospy.get_param("advertise_nav_topic","shared_nav_topic")
		'''Internal Var'''
		self.change1 = 	self.change2 = self.change3 = self.change4 = False
		self.pose_user = Odometry()
		self.pose_nav = Odometry()
		self.path = Path()
		self.goal = PoseStamped()
		self.shared_nav_mode = False
        	'''Subscribers'''
		self.sub_pose_user = self.rospy.Subscriber(self.pose_user_topic, Odometry, self.callback_pose_user) 
       		self.sub_odom = self.rospy.Subscriber(self.odom_topic, Odometry, self.callback_odom)
       		self.sub_plan_poses = self.rospy.Subscriber(self.plan_poses_topic, Path, self.callback_path)
		self.sub_nav_goal = self.rospy.Subscriber(self.nav_goal_topic, PoseStamped, self.callback_nav_goal)
		'''Publishers'''
		self.pub_advertise_nav = self.rospy.Publisher(self.advertise_nav_topic, Bool, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node("SharedNavPub", anonymous = True)
		self.rate = self.rospy.Rate(20)
		self.main()

	def callback_pose_user(self,msg):
		self.pose_user = msg
		self.change1 = True
		return

	def callback_odom(self,msg):
		self.pose_nav = msg
		self.change2 = True
		return

	def callback_path(self, msg):
		self.path = msg
		self.nextPose = self.path.poses[2].pose
		self.change3 = True
		return

	def callback_nav_goal(self, msg):
		self.goal = msg
		self.change4 = True
		return

	def main(self):
		rotMapOdom = int(input("Ingrese rotacion entre map y odom"))*np.pi/180
		while not(self.rospy.is_shutdown()):
			if self.change1 and self.change2 and self.change3 and self.change4:
				nextPoseX, nextPoseY  = self.nextPose.position.x, self.nextPose.position.y
				userX, userY = self.pose_user.pose.pose.position.x, self.pose_user.pose.pose.position.y
				distanceToNextPose = np.sqrt((nextPoseX-userX)**2+(nextPoseY-userY)**2)
				thetaWindow = np.arctan(0.25/distanceToNextPose)
				thetaNav = np.arccos(self.nextPose.orientation.w)*2
				thetaWalker = np.arccos(self.pose_nav.pose.pose.orientation.w)*2 + rotMapOdom
				thetaUser = np.arccos(self.pose_user.pose.pose.orientation.w)*2 + rotMapOdom
				print("thetaWalker",thetaWalker,"thetaNav",thetaNav,"thetaWindow",thetaWindow, "thetaUser",thetaUser)
				print(((thetaWalker+thetaNav)-thetaWindow)/(np.pi*2), " <= ", thetaUser/(np.pi*2), " <= ", ((thetaWalker+thetaNav)+thetaWindow)/(np.pi*2))
				if ((thetaWalker+thetaNav)-thetaWindow)/(np.pi*2) <= thetaUser <= ((thetaWalker+thetaNav)+thetaWindow)/(np.pi*2):
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
