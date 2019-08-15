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
		self.aux_cmd_vel_topic = self.rospy.get_param("aux_cmd_vel_topic", "/usr_cmd_vel")
		self.frc_topic = self.rospy.get_param("frc_topic","/frc")
		self.acontroller_rate = self.rospy.get_param("acontroller_rate",30)
		# m = 4
		self.controller_params = {
					"m": self.rospy.get_param("mass",0.1),
					"b_l": self.rospy.get_param("lineal_damping_c",20),
					"k_l": self.rospy.get_param("lineal_stiffness_c",11),
					"j": self.rospy.get_param("inertia", 1),
					"b_a": self.rospy.get_param("angular_damping_c", 15),
					"k_a": self.rospy.get_param("angular_stiffness_c",0.7),
					"Ts": self.rospy.get_param("Ts",1.0/self.acontroller_rate)}
		'''Subscribers'''
		self.sub_frc = self.rospy.Subscriber(self.frc_topic,Wrench,self.callback_frc)
		'''Publishers'''
		self.pub_aux_cmd_vel = self.rospy.Publisher(self.aux_cmd_vel_topic, Twist, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node("Admitance_Controller", anonymous = True)
		self.rate = self.rospy.Rate(self.acontroller_rate)
		self.vel = Twist()
		self.frc = 0
		self.trq = 0
		lnum = np.array([1, 0])
		lden = [self.controller_params["m"], self.controller_params["b_l"], self.controller_params["k_l"]]
		anum = np.array([1, 0])
		aden = [self.controller_params["j"], self.controller_params["b_a"], self.controller_params["k_a"]]
		[lnum_d, lden_d] = sg.bilinear(lnum, lden, self.acontroller_rate)
		[anum_d, aden_d] = sg.bilinear(anum, aden, self.acontroller_rate)
		self.systems = {	"linear": sg.TransferFunction(lnum_d,lden_d,dt = self.controller_params["Ts"]),
							"angular": sg.TransferFunction(anum_d,aden_d,dt = self.controller_params["Ts"])}
		self.change = self.enabled = False
		self.signal_in = {"frc": [],
					  	  "trq": [],}
		self.main_controller()

	def get_response(self):
		linear = sg.dlsim(self.systems["linear"],self.signal_in["frc"])
		angular = sg.dlsim(self.systems["angular"],self.signal_in["trq"])
		#print("v_lineal",linear[1][-1],'v_angular',angular[1][-1])
		return linear[1][-1],1.5*angular[1][-1]

	def callback_frc(self,msg):
		self.frc = msg.force.y + abs(msg.force.z)
		self.trq = msg.torque.y
		self.change = True
		if self.frc >= 0.35:
			self.enabled = True
		return

	def main_controller(self):
		min_len = int(.5/self.controller_params["Ts"])
		while not(self.rospy.is_shutdown()) and len(self.signal_in["frc"]) < min_len:
			if self.change:
				self.signal_in["frc"].append(self.frc)
				self.signal_in["trq"].append(self.trq)
				self.change = False
			self.rate.sleep()
		self.change = False
		self.vel.linear.x,self.vel.angular.z = self.get_response()
		self.pub_aux_cmd_vel.publish(self.vel)
		while not self.rospy.is_shutdown():
			#print(0)
			if self.change:
				#print(1)
				self.signal_in["frc"].pop(0)
				self.signal_in["frc"].append(self.frc)
				self.signal_in["trq"].pop(0)
				self.signal_in["trq"].append(self.trq)
				self.vel.linear.x,self.vel.angular.z = self.get_response()
				if self.vel.linear.x < 0.1:
					self.vel.linear.x = 0
				if self.enabled:
					self.pub_aux_cmd_vel.publish(self.vel)
					self.enabled = False
				self.change = False
				#print(2)
			self.rate.sleep()

if __name__ == '__main__':
	print("Starting Admitance Controller")
	try:
		ac = AdmittanceController()
	except rospy.ROSInterruptException:
		pass
		print('Exiting Admitance Controller')
