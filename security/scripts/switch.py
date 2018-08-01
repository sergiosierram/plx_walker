#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sys import stdin

class Switch():
	def __init__(self):
		self.rospy = rospy
		'''Parameters'''
		self.final_vel_topic = self.rospy.get_param("final_vel_topic","/RosAria/cmd_vel")
		self.aux_vel_topic = self.rospy.get_param("aux_vel_topic","/aux_cmd_vel")
		self.insecure_vel_topic = self.rospy.get_param("insecure_vel_topic","/insecure_cmd_vel")
		self.insecure_mode_topic = self.rospy.get_param("insecure_mode_topic","/insecure_mode")
		self.switch_rate = self.rospy.get_param("switch_rate",30)
		'''Subscribers'''
		self.sub_aux_vel = self.rospy.Subscriber(self.aux_vel_topic, Twist, self.callback_aux_vel)
		self.sub_insecure_vel = self.rospy.Subscriber(self.insecure_vel_topic, Twist, self.callback_insecure_vel)
		self.sub_insecure_mode = self.rospy.Subscriber(self.insecure_mode_topic, Bool, self.callback_insecure_mode)
		'''Publisher'''
		self.pub_final_vel = self.rospy.Publisher(self.final_vel_topic, Twist, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node('VelocitySwitch', anonymous = True)
		self.rate = self.rospy.Rate(self.switch_rate)
		self.aux_vel = self.insecure_vel = Twist()
		self.insecure_mode = False
		self.change = self.change_aux_vel = False
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

	def callback_insecure_vel(self, data):
		#msg = self.vel_format(data)
		self.insecure_vel = data
		self.change = True
		return

	def callback_insecure_mode(self, msg):
		self.insecure_mode = msg.data
		self.change = True
		return

	def main_switch(self):
		while not self.rospy.is_shutdown():
			if self.change:
				if self.insecure_mode == True:
					if self.aux_vel.linear.x > self.insecure_vel.linear.x:
						print("Limiting Velocity")
						self.vel.linear.x = self.insecure_vel.linear.x
						self.vel.angular.x = self.aux_vel.angular.x
						self.vel.angular.y = self.aux_vel.angular.y
						self.vel.angular.z = self.aux_vel.angular.z
						self.pub_final_vel.publish(self.vel)
					else:
						if self.change_aux_vel:
							print("Free Velocity - Insecure Zone")
							self.pub_final_vel.publish(self.aux_vel)
				else:
					if self.change_aux_vel:
						print("Free Velocity")
						self.pub_final_vel.publish(self.aux_vel)
				self.change = self.change_aux_vel = False
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = Switch()
	except rospy.ROSInterruptException:
		pass
