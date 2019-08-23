#!/usr/bin/python
import rospy
import numpy as np
import struct
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sys import stdin

class PlxSecurity():
	def __init__(self, name):
		self.name = name
		#Call of Initializer Functions
		self.initNode()
		self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		self.initVariables()
		#Call of main loop
		self.main_security()

	def initNode(self):
		#Node registration
		self.rospy = rospy
		self.rospy.init_node(self.name, anonymous = True)
		self.rospy.loginfo("[%s] Starting PlxSecurity Adaptive", self.name)

	def initParameters(self):
		'''Parameters Inicialization '''
		self.lrf_topic = self.rospy.get_param("laser_topic", "/scan")
		self.security_rate = self.rospy.get_param("security_rate", 30)

	def initSubscribers(self):
		'''Subscribers'''
		self.sub_lrf = self.rospy.Subscriber(self.lrf_topic, LaserScan, self.callback_lrf)

	def initPublishers(self):
		'''Publishers'''

	def initVariables(self):
		self.change = {"lrf_top": False, "lrf_floor": False, "sonar": False}
		self.lrf_top = {"angle_min":0, "angle_max":0, "angle_inc":0, "ranges": [], "offset": 0}
		self.rate = self.rospy.Rate(self.security_rate)				#Node rate configuration

	def polar_2_cartesian(self,laser_params):
		'''Function for polar coordinates to cartesian coordinates conversion'''
		angles = np.linspace(laser_params["angle_min"],laser_params["angle_max"],len(laser_params["ranges"]))	#Array of laser angles
		ranges = np.array(laser_params["ranges"])			#Array of laser ranges
		ranges[np.nonzero(ranges == -1)] = float("NaN")		#Discarding of not useful values
		y = ranges*np.sin(angles)							#Y coordinate calculation
		x = ranges*np.cos(angles) - laser_params["offset"] 	#X coordinate calculation. Offset is already contemplated by RosAria
		data = np.array([[xi,yi] for xi, yi in zip(x,y)])
		return data										#X, Y coordinates

	def is_inside(self,x,y,p1,p2):
		'''Function to determine wich laser point within a validation zone is the closest one to the robot.
		The validation zone is rectangle defined by p1 and p2'''
		x_coor = [p1[0],p2[0]]				#X coordinates of p1 and p2 points
		y_coor = [p1[1],p2[1]]				#Y coordinates of p1 and p2 points
		is_inside_x = np.zeros(x.shape)		#Auxiliary array to store the x coordinates of the point in the validation zone
		is_inside_y = np.zeros(y.shape)		#Auxiliary array to store the y coordinates of the point in the validation zone
		is_inside_xy = np.zeros(x.shape)	#Auxiliary array to validate how many points yield in the validation zone
		is_inside_x = np.where(np.logical_and(x>=min(x_coor),x<=max(x_coor)),1,0)	#Finding of x coordinates in the validation zone
		is_inside_y = np.where(np.logical_and(y>=min(y_coor),y<=max(y_coor)),1,0) 	#Finding of y coordinates in the validation zone
		indices = np.logical_and(is_inside_x==1,is_inside_y==1)						#Extracting of the indices of the points in the validation zone
		is_inside_xy[indices] = 1								#Updating of auxiliary array.
		if np.sum(is_inside_xy)>0:								#Condition to validate the presence of at least one point in the validation zone
			is_inside = True									#Checking variable of points inside the zone
			d = np.amin(np.sqrt(x[indices]**2+y[indices]**2))	#Calculation of the minimum point distance to the robot
		else:
			is_inside = False									#Checking variable meaning no points inside the zone
			d = -1												#Negative distance meaning no points inside the boz
		return is_inside,d

	def callback_lrf(self, msg):
		'''Callback function invoked when a message of the top lrf is received'''
		self.lrf_top["angle_min"] = msg.angle_min
		self.lrf_top["angle_max"] = msg.angle_max
		self.lrf_top["angle_inc"] = msg.angle_increment
		self.lrf_top["ranges"] = msg.ranges
		self.change["lrf_top"] = True
		return

	def process_lrf_data(self, lrf_data):
		data = self.polar_2_cartesian(lrf_data)
		data = data[~np.isnan(data).any(axis=1)]
		data = data[~np.isinf(data).any(axis=1)]
		try:
			clusters_raw = DBSCAN(eps=0.5, min_samples=10).fit(data)
			labels = clusters_raw.labels_
		except:
			self.rospy.logwarn("[%s] Not able to perform clustering", self.name)
			return
		n_clusters = np.max(labels)
		clusters_filt = np.array([ data[labels == i] for i in range(n_clusters + 1)])
		clusters_mean = np.array([np.mean(cluster, axis = 0) for cluster in clusters_filt])
		clusters_dist = np.array([np.sqrt(np.power(mean[0],2) + np.power(mean[1],2)) for mean in clusters_mean])
		dist=3.0
		clusters_final = np.array([ clusters_filt[k] for k, d in enumerate(clusters_dist) if d <= dist])
		for cluster in clusters_final:
			plt.scatter(cluster[:, 0], cluster[:, 1], c="k", marker = "*")
		#print(clusters_mean)
		cmap = plt.cm.get_cmap('hsv', n_clusters)
		for i in range(n_clusters+1):
			cluster = data[labels == i]
			plt.scatter(cluster[:,0],cluster[:,1], c=cmap(i), marker='.')
			plt.scatter(clusters_mean[i,0],clusters_mean[i,1], c=cmap(i), marker='x')
		plt.xlim([-20, 20])
		plt.ylim([-20, 20])
		self.fig.canvas.draw()
		return

	def main_security(self):
		self.fig = plt.gcf()
		self.fig.show()
		while not self.rospy.is_shutdown():
			if self.change["lrf_top"]:
				self.process_lrf_data(self.lrf_top)
				pass
				plt.clf()
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sec = PlxSecurity('PlxSecurity_Adaptive')
	except rospy.ROSInterruptException:
		pass
