#!/usr/bin/python
import rospy
import numpy as np
import struct
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
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
		self.rospy.loginfo("[%s] Starting PlxSecurity", self.name)

	def initParameters(self):
		'''Parameters Inicialization '''
		self.aux_cmd_vel_topic = self.rospy.get_param("aux_cmd_vel_topic", "/aux_cmd_vel")
		self.insecure_vel_topic = self.rospy.get_param("insecure_vel_topic","/insecure_cmd_vel")
		self.insecure_mode_topic = self.rospy.get_param("insecure_mode_topic","/insecure_mode")
		self.lrf_top_topic = self.rospy.get_param("laser_top_topic", "/RosAria/S3Series_1_laserscan")
		self.lrf_floor_topic = self.rospy.get_param("sonar_env_topic", "/scan")
		self.sonar_topic = self.rospy.get_param("sonar_env_topic", "/RosAria/sonar")
		self.use_sensor = {"lrf_top": self.rospy.get_param("use_lrf_top", True),
						   "lrf_floor": self.rospy.get_param("use_lrf_top", False),
						   "sonar" : self.rospy.get_param("use_sonar", True)}				#ROS Parameters: what sensors are going to be used
		self.stop_distance = {"lrf": self.rospy.get_param("stop_distance_lrf", 0.3),
							  "sonar": self.rospy.get_param("stop_distance_sonar", 0.25)}	#ROS Parameters: Initial distance at which velocity will be limited
		self.slow_distance = {"lrf": self.rospy.get_param("slow_distance_lrf", 0.6),
							  "sonar": self.rospy.get_param("slow_distance_sonar", 0.35)}	#ROS Parameters: Final distance at which velocity will be disabled
		self.slow_speed = {"lrf": self.rospy.get_param("slow_speed", 0.3),
						   "sonar": self.rospy.get_param("slow_speed_sonar", 0.1)}			#ROS Parameters: Maximum speed at velocity limitation zone
		self.robot_params = {"w": self.rospy.get_param("robot_width", 0.5),
							 "l": self.rospy.get_param("robot_length", 0.696)}				#ROS Parameters: Robot geometry parameters
		self.width_ratio = self.rospy.get_param("width_ratio", 1.5)							#ROS Parameter: Scaling factor of velocity limitation zone
		self.security_rate = self.rospy.get_param("security_rate", 30)

	def initSubscribers(self):
		'''Subscribers'''
		self.sub_aux_cmd_vel = self.rospy.Subscriber(self.aux_cmd_vel_topic, Twist,self.callback_aux_cmd_vel)	#Definition of aux velocity subscriber
		if self.use_sensor["lrf_top"]: 																		#Conditionated subscription to top lrf topic
			self.sub_lrf_top = self.rospy.Subscriber(self.lrf_top_topic, LaserScan, self.callback_lrf_top)	#Definition of top lrf subscriber
		if self.use_sensor["lrf_floor"]:																	#Conditionated subscription to floor lrf topic
			self.sub_lrf_floor = self.rospy.Subscriber(self.lrf_floor_topic, LaserScan, self.callback_lrf_floor) #Definition of floor lrf subscriber
		if self.use_sensor["sonar"]:																		#Conditionated subscription to sonar
			self.sub_sonar = self.rospy.Subscriber(self.sonar_topic, PointCloud, self.callback_sonar)		#Definition of sonar subscriber

	def initPublishers(self):
		'''Publishers'''
		self.pub_insecure_vel = self.rospy.Publisher(self.insecure_vel_topic, Twist, queue_size = 10)		#Definition of insecure vel publisher
		self.pub_insecure_mode = self.rospy.Publisher(self.insecure_mode_topic, Bool, queue_size = 10)		#Definition of insecure mode publisher

	def initVariables(self):
		self.sonar_offset_front_x = 0.331													#Distance in mm at which the sonar is located
		self.cmd_vel_x = 0											#Inicialization of velocity value
		self.vel_insecure = Twist()									#Velocity definition as Twist value linear[x, y, z] angular[x, y, z]
		self.change = {"lrf_top": False, "lrf_floor": False, "sonar": False}				#Variables for validation of new incoming data
		self.lrf_top = {"angle_min":0, "angle_max":0, "angle_inc":0, "ranges": [], "offset": 0}				#Attributes of the upper LRF data message
		self.lrf_floor = {"angle_min": 0, "angle_max": 0, "angle_inc": 0, "ranges": [], "offset": 0}		#Attributes of the floor LRF data message
		self.rate = self.rospy.Rate(self.security_rate)				#Node rate configuration

	def polar_2_cartesian(self,laser_params):
		'''Function for polar coordinates to cartesian coordinates conversion'''
		angles = np.linspace(laser_params["angle_min"],laser_params["angle_max"],len(laser_params["ranges"]))	#Array of laser angles
		ranges = np.array(laser_params["ranges"])			#Array of laser ranges
		ranges[np.nonzero(ranges == -1)] = float("NaN")		#Discarding of not useful values
		y = ranges*np.sin(angles)							#Y coordinate calculation
		x = ranges*np.cos(angles) - laser_params["offset"] 	#X coordinate calculation. Offset is already contemplated by RosAria
		return x,y											#X, Y coordinates

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

	def callback_aux_cmd_vel(self, msg):
		'''Callback function invoked when a message of the auxiliary velocity is received'''
		self.cmd_vel_x = msg.linear.x 						#Storage of linear velocity in x direction
		return

	def callback_lrf_top(self, msg):
		'''Callback function invoked when a message of the top lrf is received'''
		self.lrf_top["angle_min"] = msg.angle_min
		self.lrf_top["angle_max"] = msg.angle_max
		self.lrf_top["angle_inc"] = msg.angle_increment
		self.lrf_top["ranges"] = msg.ranges
		self.change["lrf_top"] = True
		return

	def callback_lrf_floor(self, msg):
		self.lrf_floor["angle_min"] = msg.angle_min
		self.lrf_floor["angle_max"] = msg.angle_max
		self.lrf_floor["angle_inc"] = msg.angle_increment
		self.lrf_floor["ranges"] = msg.ranges
		self.change["lrf_floor"] = True
		return

	def callback_sonar(self,msg):
		self.sonar_front_x = {"left": float(msg.points[0].x)-self.sonar_offset_front_x,
							  "right": float(msg.points[3].x)-self.sonar_offset_front_x}
		self.change["sonar"] = True
		return

	def new_distances(self):
		v, slw, wdth = self.cmd_vel_x, 0.6, 1.5
		if v <= 0.4:
			slw = 0.8
			wdth = 1.1
		elif v <= 0.5:
			slw = 1.0
			wdth = 1.2
		elif v <= 0.6:
			slw = 1.2
			wdth = 1.3
		elif v <= 0.8:
			slw = 1.4
			wdth = 1.5
		else:
			slw = 2.0
			wdth = 1.5
		return slw, wdth

	def main_security(self):
		insecure = {"lrf_top": False, "lrf_floor": False, "sonar": False}
		dist = {"lrf_top": 0, "lrf_floor": 0, "sonar": 0}
		velmax = {"lrf_top": 0,	"lrf_floor": 0, "sonar": 0}
		while not self.rospy.is_shutdown():
			rstop = {"lrf": min(self.slow_distance["lrf"], self.stop_distance["lrf"]),
					 "sonar": min(self.slow_distance["sonar"], self.stop_distance["sonar"])}
			rslow = {"lrf": max(self.slow_distance["lrf"], self.stop_distance["lrf"]),
					 "sonar": max(self.slow_distance["sonar"], self.stop_distance["sonar"])}
			if self.cmd_vel_x > 0.3:
				rslow["lrf"], self.width_ratio = self.new_distances()
			else:
				rstop["lrf"] = min(self.slow_distance["lrf"], self.stop_distance["lrf"])
				rslow["lrf"] = max(self.slow_distance["lrf"], self.stop_distance["lrf"])
				self.width_ratio = 1.2
			box_points = {
					      "top": ((-self.robot_params["l"]/2,-self.width_ratio*self.robot_params["w"]/2),(rslow["lrf"],self.width_ratio*self.robot_params["w"]/2)),
					      "floor": ((0,-self.width_ratio*self.robot_params["w"]/2),(rslow["lrf"],self.width_ratio*self.robot_params["w"]/2))
					     }
			#print(rstop["lrf"],rslow["lrf"])
			if self.change["sonar"]:
				dist["sonar"] = min(self.sonar_front_x["left"], self.sonar_front_x["right"])
				if dist["sonar"] < rstop["sonar"]:
					velmax["sonar"] = 0
					insecure["sonar"] = True
				elif dist["sonar"] < rslow["sonar"]:
					velmax["sonar"] = self.slow_speed["sonar"]*(dist["sonar"] - rstop["sonar"])/(rslow["sonar"]-rstop["sonar"])
					insecure["sonar"] = True
				else:
					insecure["sonar"] = False
#				print("Sonar: ",insecure["sonar"], dist["sonar"], velmax["sonar"])
			if self.change["lrf_top"]:
				x,y = self.polar_2_cartesian(self.lrf_top)
				inside,dist["lrf_top"] = self.is_inside(x,y,box_points["top"][0],box_points["top"][1])
				if inside and dist["lrf_top"] < rstop["lrf"]:
					velmax["lrf_top"] = 0
					insecure["lrf_top"] = True
				elif inside and dist["lrf_top"] < rslow["lrf"]:
					velmax["lrf_top"] = self.slow_speed["lrf"]*(dist["lrf_top"] - rstop["lrf"])/(rslow["lrf"]-rstop["lrf"])
					insecure["lrf_top"] = True
				else:
					insecure["lrf_top"] = False
#				print("LRF Top: ",insecure["lrf_top"], dist["lrf_top"], velmax["lrf_top"], rstop["lrf"], rslow["lrf"])
			if self.change["lrf_floor"]:
				x,y = self.polar_2_cartesian(self.lrf_floor)
				inside,dist["lrf_floor"] = self.is_inside(box_points["floor"][0],box_points["floor"][1])
				if is_inside and dist["lrf_floor"] < rstop["lrf"]:
					velmax["lrf_floor"] = 0
					insecure["lrf_floor"] = True
				elif inside and dist["lrf_floor"] < rslow["lrf"]:
					velmax["lrf_floor"] = self.slow_speed["lrf"]*(dist["lrf_floor"] - rstop["lrf"])/(rslow["lrf"]-rstop["lrf"])
					insecure["lrf_floor"] = True
				else:
					insecure["lrf_floor"] = False
#				print("LRF Floor: ",insecure["lrf_floor"], dist["lrf_floor"], velmax["lrf_floor"])
			if insecure["lrf_top"] or insecure["lrf_floor"] or insecure["sonar"]:
				vel_max_out =  [velmax["lrf_top"],velmax["lrf_floor"],velmax["sonar"]]
				#distances = [dist["lrf_top"],dist["lrf_floor"],100]
				distances = [dist["lrf_top"],dist["lrf_floor"],dist["sonar"]]
				use_sensors = [self.use_sensor["lrf_top"], self.use_sensor["lrf_floor"], self.use_sensor["sonar"]]
				rdistances = [d for c,d in enumerate(distances) if use_sensors[c]]
				index = rdistances.index(min(rdistances))
				#print(rdistances,index)
				self.pub_insecure_mode.publish(True)
				self.vel_insecure.linear.x = vel_max_out[index]
				self.pub_insecure_vel.publish(self.vel_insecure)
				self.change["lrf_top"] = False
				self.change["lrf_floor"] = False
				self.change["sonar"] = False
			else:
				self.pub_insecure_mode.publish(False)
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sec = PlxSecurity('PlxSecurity')
	except rospy.ROSInterruptException:
		pass
