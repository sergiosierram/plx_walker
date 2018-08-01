#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Wrench

if __name__ == '__main__':
	try:
	 	pub_left = rospy.Publisher('/frc_left', Wrench, queue_size = 10)
		pub_right = rospy.Publisher('/frc_right', Wrench, queue_size = 10)
		frc_left = Wrench()
		frc_right = Wrench()
		fs = 100
		rospy.init_node('artificial_signals', anonymous = True)
		rate = rospy.Rate(fs)
		ti = rospy.get_time()
		while not rospy.is_shutdown():
			t = rospy.get_time()-ti
			frc_left.force.z = frc_right.force.z =  np.sin(2*np.pi*1*t)
			frc_left.force.y = frc_right.force.y =  np.sin(2*np.pi*1*t) + np.sin(2*np.pi*10*t+np.pi/2)
			pub_left.publish(frc_left)
			pub_right.publish(frc_right)
			rate.sleep()

	except rospy.ROSInterruptException:
		print("Something's gone wrong. Exiting")
