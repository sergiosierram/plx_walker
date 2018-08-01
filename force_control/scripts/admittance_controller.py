#!/usr/bin/python
import rospy
import numpy as np
#import scipy.signal as sg
from geometry_msgs.msg import Twist,Wrench
from std_msgs.msg import Bool
from scipy import signal as sg
from sys import stdin

class AdmittanceController(object):
	def __init__(self):
		'''Parameters Inicialization '''
		self.rospy = rospy
		self.aux_cmd_vel_topic = self.rospy.get_param("aux_cmd_vel_topic", "/aux_cmd_vel")
		self.frc_topic = self.rospy.get_param("frc_topic","/frc")
		#self.frc_topic = self.rospy.get_param("frc_topic","/linear_frc")
		#self.trq_topic = self.rospy.get_param("trq_topic","/torque")
		self.acontroller_rate = self.rospy.get_param("acontroller_rate",6)
		self.controller_params = {
					"m": self.rospy.get_param("mass",20),
					"b_l": self.rospy.get_param("ldaming_ratio",5),
					"j": self.rospy.get_param("inertia",5),
					"b_a": self.rospy.get_param("adamping_ratio",4),
					"Ts": self.rospy.get_param("Ts",1.0/self.acontroller_rate)}
		'''Subscribers'''
		self.sub_frc = self.rospy.Subscriber(self.frc_topic,Wrench,self.callback_frc)
		#self.sub_trq = self.rospy.Subscriber(self.trq_topic, Wrench,self.callback_trq)
		'''Publishers'''
		self.pub_aux_cmd_vel = self.rospy.Publisher(self.aux_cmd_vel_topic, Twist, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node("Admitance_Controller", anonymous = True)
		self.rate = self.rospy.Rate(self.acontroller_rate)
		self.vel = Twist()
		self.frc = 0
		self.trq = 0
		lnum = np.ones(2)*self.controller_params["Ts"]/2
		lden = [self.controller_params["m"]+self.controller_params["b_l"],self.controller_params["b_l"]-self.controller_params["m"]]
		anum = np.ones(2)*self.controller_params["Ts"]/2
		aden = [self.controller_params["j"]+self.controller_params["b_a"],self.controller_params["b_a"]-self.controller_params["j"]]
		self.systems = {
				"linear": sg.TransferFunction(lnum,lden,dt = self.controller_params["Ts"]/2),
				"angular": sg.TransferFunction(anum,aden,dt = self.controller_params["Ts"]/2)
				}
		#self.change = {"frc": False,
		#			   "trq": False}
		self.change = False
		self.signal_in = {"frc": [],
					  	  "trq": [],
					  	  "t": np.arange(0,1,self.controller_params["Ts"])}
		self.main_controller()

	def get_responce(self):
		linear = sg.dlsim(self.systems["linear"],self.signal_in["frc"])#,signal_in["t"]),
		#if np.mean(self.signal_in["trq"])>=0:
		angular = sg.dlsim(self.systems["angular"],self.signal_in["trq"])#,signal_in["t"])
		#else:
		#	print('Torque Negativo')
		#	angular = sg.dlsim(self.systems["angular"],np.absolute(self.signal_in["trq"]))#,signal_in["t"])
		#	print(angular)
		#	angular = angular*-1
		#	print(angular)
		print(linear[1][-1],angular[1][-1])
		return 2*linear[1][-1],2*angular[1][-1]

#	def callback_frc(self,msg):
#		self.frc = msg.force.y
#		self.change["frc"] = True
#		return

#	def callback_trq(self,msg):
#		self.trq = msg.torque.y
#		self.change["trq"] = True
#		return
	def callback_frc(self,msg):
		self.frc = msg.force.y
		self.trq = msg.torque.y
		self.change = True

	def main_controller(self):
		#signal_in = {"frc": [],
		#			 "trq": [],
		#			 "t": np.arange(0,1,self.controller_params["Ts"])}
		min_len = int(.5/self.controller_params["Ts"])
		while not(self.rospy.is_shutdown()) and len(self.signal_in["frc"]) < min_len:
			#if self.change["frc"]:
			#	signal_in["frc"].append(self.frc)
			#	self.change["frc"] = False
			#if self.change["trq"]:
			#	signal_in["trq"].append(self.trq)
			#	self.change["trq"] = False
			if self.change:
				self.signal_in["frc"].append(self.frc)
				self.signal_in["trq"].append(self.trq)
				self.change = False
			self.rate.sleep()
		self.change = False
		self.vel.linear.x,self.vel.angular.z = self.get_responce()
		self.pub_aux_cmd_vel.publish(self.vel)
		print('s')
		while not self.rospy.is_shutdown():
			#if self.change["frc"]:
			#	signal_in["frc"].pop(0)
			#	signal_in["frc"].append(self.frc)
			#if self.change["trq"]:
			#	signal_in["trq"].pop(0)
			#	signal_in["trq"].append(self.trq)
			#if self.change["frc"] and self.change["trq"]:
			#	print('p')
			#	print(signal_in)
			#	self.vel.linear.x,self.vel.angular.z = self.get_responce(self.systems, signal_in)
			#	#self.vel.linear.x,self.vel.angular.x = self.vel.linear.y,self.vel.angular.y
			#	self.pub_aux_cmd_vel.publish(self.vel)
			#	self.change["frc"] = False
			#	self.change["trq"] = False
			if self.change:
				self.signal_in["frc"].pop(0)
				self.signal_in["frc"].append(self.frc)
				self.signal_in["trq"].pop(0)
				self.signal_in["trq"].append(self.trq)
				print('p')
				self.vel.linear.x,self.vel.angular.z = self.get_responce()
				self.pub_aux_cmd_vel.publish(self.vel)
				self.change = False
			#	self.change["trq"] = False
			self.rate.sleep()

if __name__ == '__main__':
	print('a')
	try:
		ac = AdmittanceController()
	except rospy.ROSInterruptException:
		pass
		print('a')
