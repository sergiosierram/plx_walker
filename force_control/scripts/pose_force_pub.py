#!/usr/bin/python
import rospy
import numpy as np
#import scipy.signal as sg
from geometry_msgs.msg import Twist, Wrench, Pose
from nav_msgs.msg import Odometry

class ForcePosePub(object):
	def __init__(self):
		'''Parameters Inicialization '''
		self.rospy = rospy
		self.frc_topic = self.rospy.get_param("frc_topic","/frc")
		self.pose_topic = self.rospy.get_param("pose_topic","/pose_force")
		self.odom_topic = self.rospy.get_param("odom_topic","/RosAria/pose")
		self.change1 = 	self.change2 = False
        	'''Subscribers'''
		self.sub_frc = self.rospy.Subscriber(self.frc_topic, Wrench, self.callback_frc)
       		self.sub_odom = self.rospy.Subscriber(self.odom_topic, Odometry, self.callback_odom)
        	#self.sub_trq = self.rospy.Subscriber(self.trq_topic, Wrench,self.callback_trq)
		'''Publishers'''
		self.pub_pose = self.rospy.Publisher(self.pose_topic, Odometry, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node("ForcePosePub", anonymous = True)
		self.rate = self.rospy.Rate(20)
		self.frc = 0
		self.trq = 0
		self.pose = Odometry()
		self.pose_nav = Odometry()
		self.qz = 0
		self.qw_user = 0
		self.qz_user = 0
		self.quaternion = [0,0,0,0]
		self.odom_position = 0
		self.main()

	def callback_frc(self,msg):
		self.frc = msg.force.y
		self.trq = msg.torque.y
		self.yaw = (90*self.trq/70)*np.pi/180
		print(self.yaw*180/np.pi)
		if self.change2:
			angle = (np.arccos(self.qw)*2)+np.sign(self.qz)*self.yaw
			self.qw_user = np.cos(angle/2)
			self.qz_user = np.sign(self.qz)*np.sin(angle*0.5)
			#print(self.qw, self.yaw, angle)
		self.change1 = True
		return

	def callback_odom(self,msg):
		self.pose_nav = msg
		self.qw = self.pose_nav.pose.pose.orientation.w
		self.qz = self.pose_nav.pose.pose.orientation.z
		#print(self.pose.pose.pose.orientation.z)
		self.change2 = True
		return

	def make_message(self):
		self.pose.header = self.pose_nav.header
		self.pose.child_frame_id = self.pose_nav.child_frame_id
		self.pose.pose.pose.position = self.pose_nav.pose.pose.position
		self.pose.pose.pose.orientation.x = 0
		self.pose.pose.pose.orientation.y = 0
		self.pose.pose.pose.orientation.z = self.qz_user
		self.pose.pose.pose.orientation.w = self.qw_user
		return

	def main(self):
		while not(self.rospy.is_shutdown()):
			if self.change1 and self.change2:
				self.make_message()
				self.pub_pose.publish(self.pose)
				self.change = False
			self.rate.sleep()


if __name__ == '__main__':
	print('a')
	try:
		ac = ForcePosePub()
	except rospy.ROSInterruptException:
		pass
		print('b')
