#!/usr/bin/python
import rospy
import numpy as np
from include.flc_filters.FLC_Filters import *
from geometry_msgs.msg import Wrench

class FLCFilters(object):
	def __init__(self):
		'''Parameters Inicialization '''
		self.rospy = rospy
		self.frc_topic = self.rospy.get_param("frc_topic","/frc")
		self.frc_left_topic = self.rospy.get_param("frc_left","/frc_left")
		self.frc_right_topic = self.rospy.get_param("frc_right","/frc_right")
		self.wflc_params = {"M": self.rospy.get_param("wflc_order",1),
			"mu": self.rospy.get_param("wflc_amplitude_a_g",0.05),
			"mu0": self.rospy.get_param("wflc_frequency_a_g",0.005),
			"mub": self.rospy.get_param("wflc_bias_a_g",0.05),
			"sum_w0": 0,
			"w0": np.random.rand(1),
			"X": [],
			"W": [],
			"Wb": 0,
			"fs": 100}
		self.r_flc_params = {"M": self.rospy.get_param("r_flc_order",1),
		     "mu": self.rospy.get_param("r_flc_amplitude_a_g",0.008),
		     "X": [],
		     "W": [],
			 "w0": 2*np.pi*np.random.rand(1),
			 "k": 1,
			 "fs": 100}
		self.l_flc_params = {"M": self.rospy.get_param("l_flc_order",1),
		   	 "mu": self.rospy.get_param("l_flc_amplitude_a_g",0.008),
		   	 "X": [],
		   	 "W": [],
			 "w0": 2*np.pi*np.random.rand(1),
			 "k": 1,
			 "fs": 100}
		'''Subscribers'''
		#self.sub_frc = self.rospy.Subscriber(self.frc_topic,Wrench,self.callback_frc)
		self.pub_left = self.rospy.Subscriber(self.frc_left_topic, Wrench, self.callback_frc_left)
		self.pub_right = self.rospy.Subscriber(self.frc_right_topic, Wrench, self.callback_frc_right)
		'''Publishers'''
		self.pub_frc = rospy.Publisher(self.frc_topic,Wrench, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node("FLC_Filters", anonymous = True)
		self.frc = Wrench()
		self.frc_left = Wrench()
		self.frc_right = Wrench()
		self.change = {"left": False,
					   "right": False}
		self.wflc_params["W"] = np.zeros(self.wflc_params["M"]*2)
		self.l_flc_params["W"] = np.zeros(self.l_flc_params["M"]*2)
		self.r_flc_params["W"] = np.zeros(self.r_flc_params["M"]*2)
		self.rospy.spin()

	def callback_frc_left(self,msg):
		self.frc_left = msg
		if self.change["right"]:
			self.main_callback()
		else:
			self.change["left"] = True
		return

	def callback_frc_right(self,msg):
		self.frc_right = msg
		if self.change["left"]:
			self.main_callback()
		else:
			self.change["right"] = True
		return

	def main_callback(self):
		fz = self.frc_left.force.z + self.frc_right.force.z
		tremor,self.wflc_params = wflc(fz,self.wflc_params)
#		print(self.wflc_params["w0"])
		self.l_flc_params["w0"] = self.wflc_params["w0"]*self.wflc_params["fs"]
		self.r_flc_params["w0"] = self.wflc_params["w0"]*self.wflc_params["fs"]
		left_tremor,self.l_flc_params = flc(self.frc_left.force.y,self.l_flc_params)
		right_tremor,self.r_flc_params = flc(self.frc_left.force.y,self.r_flc_params)
		"""
		self.frc.force.x = self.wflc_params["w0"]
		self.frc.force.y = tremor
		self.frc.force.z = fz
		"""
		self.frc_left.force.y = self.frc_left.force.y - left_tremor
		self.frc_right.force.y = self.frc_right.force.y - right_tremor
		self.frc.force.y = self.frc_left.force.y# + self.frc_right.force.y
		self.frc.force.x = self.frc_right.force.y#self.frc_left.force.y - self.frc_right.force.y
		self.pub_frc.publish(self.frc)
		self.change["left"] = False
		self.change["right"] = False

if __name__ == '__main__':
	try:
		flc_filters = FLCFilters()
	except rospy.ROSInterruptException:
		print("Something's gone wrong. Exiting")
