#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sys import stdin

class Switch():
	def __init__(self):
		self.rospy = rospy
		self.rospy.init_node('VelocitySwitch', anonymous = True)
		'''Parameters'''
		self.final_vel_topic = self.rospy.get_param("~final_vel_topic","/RosAria/cmd_vel")
		self.aux_vel_topic = self.rospy.get_param("~aux_vel_topic","/aux_cmd_vel")
		self.usr_vel_topic = self.rospy.get_param("~usr_cmd_vel_topic", "/usr_cmd_vel")
		self.nav_vel_topic = self.rospy.get_param("~nav_cmd_vel_topic", "/nav_cmd_vel")
		self.insecure_vel_topic = self.rospy.get_param("~insecure_vel_topic","/insecure_cmd_vel")
		self.insecure_mode_topic = self.rospy.get_param("~insecure_mode_topic","/insecure_mode")
		self.shared_mode_topic = self.rospy.get_param("~shared_mode_topic", "/shared_mode")
		self.shared_mode = self.rospy.get_param("~shared_mode", False)
		self.switch_rate = self.rospy.get_param("~switch_rate", 20)
		'''Subscribers'''
		self.sub_aux_vel = self.rospy.Subscriber(self.aux_vel_topic, Twist, self.callback_aux_vel)
		self.sub_usr_vel = self.rospy.Subscriber(self.usr_vel_topic, Twist, self.callback_usr_vel)
		self.sub_nav_vel = self.rospy.Subscriber(self.nav_vel_topic, Twist, self.callback_nav_vel)
		self.sub_insecure_vel = self.rospy.Subscriber(self.insecure_vel_topic, Twist, self.callback_insecure_vel)
		self.sub_insecure_mode = self.rospy.Subscriber(self.insecure_mode_topic, Bool, self.callback_insecure_mode)
		self.sub_shared_mode = self.rospy.Subscriber(self.shared_mode_topic, Bool, self.callback_shared_mode)
		'''Publisher'''
		self.pub_final_vel = self.rospy.Publisher(self.final_vel_topic, Twist, queue_size = 10)
		'''Node Configuration'''
		self.rate = self.rospy.Rate(self.switch_rate)
		self.aux_vel = self.usr_vel = self.insecure_vel = Twist()
		self.insecure_mode = self.enabled = False
		self.change = self.change2 = self.change_aux_vel = self.change_usr_vel = self.change_nav_vel = False
		self.published = True
		self.msg = Twist()
		self.vel = Twist()
		self.main_switch()
	'''
	def vel_format(self,data):
		self.msg.linear.x, self.msg.linear.y, self.msg.linear.z, = data.linear.x, data.linear.y, data.linear.z
		self.msg.angular.x, self.msg.angular.y, self.msg.angular.z, = data.angular.x, data.angular.y, data.angular.z
		return self.msg
	'''
	def callback_aux_vel(self, data):
		#msg = self.vel_format(data)
		self.aux_vel = data
		self.change_aux_vel = True
		return

	def callback_usr_vel(self, data):
		self.usr_vel = data
		self.change_usr_vel = True
		return

	def callback_nav_vel(self, data):
		self.nav_vel = data
		self.change_nav_vel = True
		return

	def callback_insecure_vel(self, data):
		#msg = self.vel_format(data)
		self.insecure_vel = data
		self.change = True
		return

	def callback_insecure_mode(self, msg):
		self.insecure_mode = msg.data
		self.change = True
		return

	def callback_shared_mode(self, msg):
		self.shared_status = msg.data
		self.change2 = True
		return

	def main_switch(self):
		print("-----------------------------------------------------------")
		print(self.shared_mode)
		print("-----------------------------------------------------------")
		while not self.rospy.is_shutdown():
			ref_vel = Twist()
			if self.change_aux_vel:
				ref_vel = self.aux_vel
				self.enabled = True
			else:
				if self.shared_mode:
					if self.change2 and self.change_usr_vel and self.change_nav_vel:
						if self.shared_status:
							self.rospy.loginfo("Shared mode: User Control")
							if self.usr_vel.linear.x > self.nav_vel.linear.x:
								ref_vel.linear.x = self.nav_vel.linear.x
							else:
								ref_vel.linear.x = self.usr_vel.linear.x
							if ref_vel.linear.x < 0:
								ref_vel.linear.x = 0
							ref_vel.angular.z = self.usr_vel.angular.z
						else:
							self.rospy.loginfo("Shared mode: Robot Control")
							ref_vel.linear.x = self.usr_vel.linear.x
							ref_vel.angular.z = self.nav_vel.angular.z
						self.change2 = False			
						self.enabled = True		
					'''
					else:
						print("-")
						# TODO: Add weighted velocity selection
						if self.change_usr_vel:
							ref_vel = self.usr_vel
						elif self.change_nav_vel:
							print("*")
							ref_vel = self.nav_vel
						else:
							ref_vel = Twist()
					'''
				else:
					# TODO: Add weighted velocity selection
					if self.change_usr_vel:
						ref_vel = self.usr_vel
					elif self.change_nav_vel:
						ref_vel = self.nav_vel
					else:
						ref_vel = Twist()
					self.enabled = True
			if self.change and self.enabled:
				if self.insecure_mode == True:
					if ref_vel.linear.x > self.insecure_vel.linear.x:
						self.rospy.loginfo("Limiting Velocity")
						#print("Limiting Velocity")
						self.vel.linear.x = self.insecure_vel.linear.x
						self.vel.angular.x = ref_vel.angular.x
						#self.vel.angular.y = self.aux_vel.angular.y
						self.vel.angular.z = ref_vel.angular.z
						self.pub_final_vel.publish(self.vel)
					else:
						if self.change_aux_vel or self.change_usr_vel or self.change_nav_vel:
							self.rospy.loginfo("Free Velocity - Insecure Zone")
							#print("Free Velocity - Insecure Zone")
							self.pub_final_vel.publish(ref_vel)
				else:
					if self.change_aux_vel or self.change_usr_vel or self.change_nav_vel:
						self.rospy.loginfo("Free Velocity")
						#print("Free Velocity")
						self.pub_final_vel.publish(ref_vel)
				self.change = self.change_aux_vel = self.change_usr_vel = self.change_nav_vel = False
				self.enabled = False
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = Switch()		
	except rospy.ROSInterruptException:
		pass
