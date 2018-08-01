#!/usr/bin/python
import rospy
import csv
from threading import Thread
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sys import stdin


class DataLogger():
	def __init__(self):
		self.rospy = rospy
		'''Parameters'''
		self.cmd_vel_topic = self.rospy.get_param("final_vel_topic","/RosAria/cmd_vel")
		self.aux_vel_topic = self.rospy.get_param("aux_vel_topic","/aux_cmd_vel")
		self.insecure_vel_topic = self.rospy.get_param("insecure_vel_topic","/insecure_cmd_vel")
		self.insecure_mode_topic = self.rospy.get_param("insecure_mode_topic","/insecure_mode")
		'''Subscribers'''
		self.sub_aux_vel = self.rospy.Subscriber(self.aux_vel_topic, Twist, self.callback_aux_vel)
		self.sub_insecure_vel = self.rospy.Subscriber(self.insecure_vel_topic, Twist, self.callback_insecure_vel)
		self.sub_insecure_mode = self.rospy.Subscriber(self.insecure_mode_topic, Bool, self.callback_insecure_mode)
		self.sub_cmd_vel = self.rospy.Subscriber(self.cmd_vel_topic,Twist, self.callback_cmd_vel)
		'''Node Configuration'''
		self.rospy.init_node('VelocitySwitch', anonymous = True)
		self.aux_vel = self.insecure_vel = self.cmd_vel = Twist()
		self.data = {"insecure_mode": [],
				 "insecure_vel": [],
				 "aux_cmd_vel": [],
				 "cmd_vel": []}
		self.time = {"insecure_mode": [],
				 "insecure_vel": [],
				 "aux_cmd_vel": [],
				 "cmd_vel": []}
		self.change = False
		self.exit_flag = False
		self.rate = self.rospy.Rate(10)
		self.main_dl()

	def callback_aux_vel(self, data):
		self.data["aux_cmd_vel"].append(data.linear.x)
		self.time["aux_cmd_vel"].append(rospy.get_time())
		return

	def callback_insecure_vel(self, data):
		self.data["insecure_vel"].append(data.linear.x )
		self.time["insecure_vel"].append(rospy.get_time() )
		return

	def callback_insecure_mode(self, msg):
		self.data["insecure_mode"].append(msg.data)
		self.time["insecure_mode"].append(rospy.get_time() )
		return

	def callback_cmd_vel(self, data):
		self.data["cmd_vel"].append(data.linear.x)
		self.time["cmd_vel"].append(rospy.get_time())
		return

	def main_dl(self):
		while not self.rospy.is_shutdown() and not self.exit_flag:
			self.rate.sleep()
		for i in ["aux_cmd_vel","insecure_vel","insecure_mode","cmd_vel"]:
			data = [[self.time[i][j],self.data[i][j]] for j in range(len(self.time[i]))]
			with open("/home/plxbot/catkin_ws/src/plx_security/data/"+i+".csv","w") as f:
				wr = csv.writer(f)
				wr.writerows([self.time[i],self.data[i]])

	def wait_for_exit(self):
		print("Press Enter for exit: ")
		x = stdin.readline().strip()
		self.exit_flag = True

if __name__ == '__main__':
	try:
		dl = DataLogger()
		thread1 = Thread(target = dl.wait_for_exit)
		thread1.start()
	except rospy.ROSInterruptException:
		pass
