#!/usr/bin/python
import rospy
import serial
import time
from threading import Thread 
from geometry_msgs.msg import Wrench
from sys import stdin

class FrcAcquirer():
	def __init__(self):
		self.pub_left = rospy.Publisher('frc_left', Wrench, queue_size = 10)
		self.pub_right = rospy.Publisher('frc_right', Wrench, queue_size = 10)
		self.pub_trq = rospy.Publisher('torque',Wrench, queue_size = 10)
		self.frc_left = Wrench()
		self.frc_right = Wrench()
		self.trq = Wrench()
		self.serialPort = serial.Serial("/dev/ttyACM0")
		self.serialChar = 'x'
		self.exitFlag = False
		self.fs = 10
		self.rospy = rospy
		self.rospy.init_node('frc_acquisition', anonymous = True)
		self.rate = self.rospy.Rate(self.fs)
		self.dataExport = [False, 'data.txt']
	
	def acquire(self):
		if self.dataExport[0]:
			f = open(self.dataExport[1],'w')
			ti = time.time()
		while not self.exitFlag and not rospy.is_shutdown():
			self.serialPort.write(self.serialChar)
			# data = [x_l, y_l, z_l, x_r, y_r, z_r]]		
			data = [float(i) for i in self.serialPort.readline().strip().split()]
			self.frc_left.force.x, self.frc_left.force.y, self.frc_left.force.z = data[0], data[1], data[2]
			self.frc_right.force.x, self.frc_right.force.y, self.frc_right.force.z = data[3], data[4], data[5]
			self.trq.torque.x = int(data[3])-int(data[0])
			self.trq.torque.y = int(data[4])-int(data[1])
			self.trq.torque.z = int(data[5])-int(data[2])
			if self.dataExport[0]:
				t = round(time.time()-ti,2)	
				f.write(str(t)+" "+" ".join(map(str, data))+str("\n"))
			self.pub_left.publish(self.frc_left)
			self.pub_right.publish(self.frc_right)
			self.pub_trq.publish(self.trq)
			self.rate.sleep()
		print("Exiting ...")
		if self.dataExport[0]:
			f.close()
		self.serialPort.close()

	def wait_for_exit(self):
		print("Press Enter for exit: ")
		x = stdin.readline()
		self.exitFlag = True

if __name__ == '__main__':
	try:
		frc = FrcAcquirer()
		thread1 = Thread(target = frc.wait_for_exit)
		thread1.start()
		#thread1.join()
		frc.acquire()
		#thread1.join()
	except rospy.ROSInterruptException:
		print("Something's gone wrong. Exiting")
		
		
