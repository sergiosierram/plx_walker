#!/usr/bin/python
import rospy
import time
import numpy as np
from scipy.signal import butter, lfilter
from threading import Thread
from geometry_msgs.msg import Wrench
from sys import stdin
from include.daq_controller.DAQ_v2 import DAQ_v2

class FrcAcquirer():
	def __init__(self):
		self.pub_left = rospy.Publisher('/frc_left', Wrench, queue_size = 10)
		self.pub_right = rospy.Publisher('/frc_right', Wrench, queue_size = 10)
		self.pub_frc = rospy.Publisher('/frc',Wrench, queue_size = 10)
		self.frc_left = Wrench()
		self.frc_right = Wrench()
		self.frc = Wrench()
		self.exitFlag = False
		self.fs = 15 	#Max 30. Hz
		self.tCalib = 1
		self.calibrate = True
		self.daq = DAQ_v2()
		self.rospy = rospy
		self.rospy.init_node('frc_acquisition', anonymous = True)
		self.rate = self.rospy.Rate(self.fs)
		self.filter = {"order":3, "cutoff":1.5}
		self.dataExport = [False, 'data.txt']
		self.lower_minimum = True
		self.data_frc_ly = self.data_frc_lz = self.data_frc_ry = self.data_frc_rz = []
		self.filt_ly = self.filt_lz = self.filt_ry = self.filt_rz = []
		self.filtered_data = []

	def calibration(self, t):
		initial_data = []
		tot_time = 0
		while tot_time < t:
			h3 = time.time()
			data = [float(i) for i in self.daq.get_forces().split('\t')]
			initial_data.append(data)
			h4 = time.time()
			tot_time += (h4-h3)
		initial_data = np.array(initial_data)
		return np.mean(initial_data,axis=0)

	def butter_lowpass(self, cutoff, fs, order):
		nyq = 0.5*fs
		normal_cutoff = cutoff/nyq
		b,a = butter(order, normal_cutoff, btype='low', analog=False)
		return b, a

	def butter_lowpass_filter(self, data, b, a):
		y = lfilter(b, a, data)
		return y

	def acquire(self):
		means = [0]*4
		if self.dataExport[0]:
			f = open(self.dataExport[1],'w')
			ti = time.time()
		if self.calibrate:
			means = self.calibration(self.tCalib)
		print('Calibration Done', means)
		b, a = self.butter_lowpass(self.filter["cutoff"], 10, self.filter["order"])
		while not self.exitFlag and not rospy.is_shutdown():
			print("In the loop")
			data = [float(i) for i in self.daq.get_forces().split('\t')]
			self.data_frc_ly = np.append(self.data_frc_ly, data[0]-means[0])
			self.data_frc_lz = np.append(self.data_frc_lz, data[1]-means[1])
			self.data_frc_ry = np.append(self.data_frc_ry, data[2]-means[2])
			self.data_frc_rz = np.append(self.data_frc_rz, data[3]-means[3])
			self.filt_ly = self.butter_lowpass_filter(self.data_frc_ly, b, a)
			self.filt_lz = self.butter_lowpass_filter(self.data_frc_lz, b, a)
			self.filt_ry = self.butter_lowpass_filter(self.data_frc_ry, b, a)
			self.filt_rz = self.butter_lowpass_filter(self.data_frc_rz, b, a)
			if len(self.data_frc_ly) > 100:
				self.data_frc_ly = np.delete(self.data_frc_ly,0)
				self.data_frc_lz = np.delete(self.data_frc_lz,0)
				self.data_frc_ry = np.delete(self.data_frc_ry,0)
				self.data_frc_rz = np.delete(self.data_frc_rz,0)
				self.filt_ly = np.delete(self.filt_ly,0)
				self.filt_lz = np.delete(self.filt_lz,0)
				self.filt_ry = np.delete(self.filt_ry,0)
				self.filt_rz = np.delete(self.filt_rz,0)
			# Message building
			self.frc_left.force.x, self.frc_left.force.y, self.frc_left.force.z = 0, self.filt_ly[-1], self.filt_lz[-1]
			self.frc_right.force.x, self.frc_right.force.y, self.frc_right.force.z = 0, self.filt_ry[-1], self.filt_rz[-1]
			self.frc.torque.x = 0
			self.frc.torque.y = self.frc_right.force.y - self.frc_left.force.y
			self.frc.torque.z = self.frc_right.force.z - self.frc_left.force.z
			self.frc.force.x = 0
			self.frc.force.y = self.frc_left.force.y + self.frc_right.force.y
			self.frc.force.z = self.frc_left.force.z + self.frc_right.force.z
			if self.dataExport[0]:
				t = round(time.time()-ti,2)
				f.write(str(t)+" "+" ".join(map(str, data))+str("\n"))
			self.pub_left.publish(self.frc_left)
			self.pub_right.publish(self.frc_right)
			self.pub_frc.publish(self.frc)
			self.rate.sleep()
		print("Exiting ...")
		if self.dataExport[0]:
			f.close()
		self.daq.close_port()

	def wait_for_exit(self):
		print("Press Enter for exit: ")
		x = stdin.readline().strip()
		self.exitFlag = True

	def acquire_raw(self):
		if self.calibrate:
			means = self.calibration(self.tCalib)
		print('Calibration Done', means)
		while not self.exitFlag and not rospy.is_shutdown():
			data = [float(i) for i in self.daq.get_forces().split('\t')]
			self.data_frc_ly = data[0]-means[0]
			self.data_frc_lz = data[1]-means[1]
			self.data_frc_ry = data[2]-means[2]
			self.data_frc_rz = data[3]-means[3]
			self.frc_left.force.x, self.frc_left.force.y, self.frc_left.force.z = 0, self.data_frc_ly, self.data_frc_lz
			self.frc_right.force.x, self.frc_right.force.y, self.frc_right.force.z = 0, self.data_frc_ry, self.data_frc_rz
			self.frc.torque.x = 0
			self.frc.torque.y = self.frc_right.force.y - self.frc_left.force.y
			self.frc.torque.z = self.frc_right.force.z - self.frc_left.force.z
			self.frc.force.x = 0
			self.frc.force.y = self.frc_left.force.y + self.frc_right.force.y
			self.frc.force.z = self.frc_left.force.z + self.frc_right.force.z
			self.pub_left.publish(self.frc_left)
			self.pub_right.publish(self.frc_right)
			self.pub_frc.publish(self.frc)
			self.rate.sleep()

if __name__ == '__main__':
	try:
		print("Perra")
		frc = FrcAcquirer()
		#thread1 = Thread(target = frc.wait_for_exit)
		#thread1.start()
		#frc.acquire()
		thread1 = Thread(target = frc.acquire)
		thread1.start()
		frc.wait_for_exit()
	except rospy.ROSInterruptException:
		print("Something's gone wrong. Exiting")
