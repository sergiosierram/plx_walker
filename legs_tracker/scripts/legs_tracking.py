#!/usr/bin/python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LegsTracking(object):
    def __init__ (self):
        '''Parameters Inicialization '''
        self.rospy = rospy														#Rospy object inicialization
        self.lrf_topic = self.rospy.get_param("lrf_env_topic", "/scan")			#ROS Parameter: topic name of front sonar
        self.roi = {"angle_min": self.rospy.get_param("roi_angle_min", -30*np.pi/180.0),
                    "angle_max": self.rospy.get_param("roi_angle_max", 30*np.pi/180.0),
                    "r_max": self.rospy.get_param("roi_r_max", 1000)}
        self.lgs_tracking_rate = self.rospy.get_param("lgs_tracking_rate", 10)	#ROS Parameter: Node performance rate
        '''Subscribers'''
        self.sub_lrf = self.rospy.Subscriber(self.lrf_topic, LaserScan, self.callback_lrf)                  #Definition of lrf subscriber
        '''Publishers'''
        self.pub_insecure_vel = self.rospy.Publisher(self.insecure_vel_topic, Twist, queue_size = 10)		#Definition of insecure vel publisher
        self.pub_insecure_mode = self.rospy.Publisher(self.insecure_mode_topic, Bool, queue_size = 10)		#Definition of insecure mode publisher
        '''Node Configuration'''
        self.rospy.init_node("plx_legs_tracker", anonymous = True)		#Node inicialization
        self.rate = self.rospy.Rate(self.security_rate)				#Node rate configuration
        self.change = False
        self.error = {"flag": False,
                      "count": 0}
        self.lrf = {"angles": [],
                    "ranges": [],
                    "transitions": []}				#Attributes of the floor LRF data message
        self.legs_params = {"lp": None,
                            "l" :  float('Inf')}
        self.midpoints = {"r": [],
                          "l": []}
        self.transitions = {"n": 0,
                            "indices": [],
                            "angles": [],
                            "previous": 0}
        self.state = []
        self.main_lgs_tracking()						#Call of main node function

    def callback_lrf(self,msg):
        '''Callback function invoked when a message of the top lrf is received'''
        all_ranges = msg.ranges
        all_angles = np.linspace(msg.angle_min,msg.angle_max,len(all_ranges))
        indices = np.logical_and(all_angles>=self.roi["angle_min"],all_angles<=self.roi["angle_max"])
        self.lrf["angles"] = all_angles[indices]
        self.lrf["ranges"] = all_ranges[indices]
        self.lrf["ranges"][np.nonzero(self.lrf["ranges"]>self.roi["r_max"])] = self.roi["r_max"]
        self.lrf["diff_ranges"] = abs(np.diff(self.lrf["ranges"]))
        self.change = True
        return

    def online_caliration(self):
        self.is_oc_done = True
        return

    def count_transitions(self):
        self.transitions["previous"] = self.transitions["n"]
        self.transitions["indices"] = np.nonzero(self.lrf["diff_ranges"] >= self.legs_params["l"])
        self.transitions["n"] = len(self.transitions["indices"])
        self.transitions["angles"] = self.lrf["angles"][self.transitions["indices"]]
        return

    def get_midpoint(self):
        if self.transitions["n"] == 4:
            l_mp_i = round((self.transitions["indices"][0]+self.transitions["indices"][1])/2)
            r_mp_i = round((self.transitions["indices"][2]+self.transitions["indices"][3])/2)
            l_mp = [self.lrf["angles"][l_mp_i],self.lrf["ranges"][l_mp_i]]
            r_mp = [self.lrf["angles"][r_mp_i],self.lrf["ranges"][r_mp_i]]
        elif self.transitions["n"] == 3:
            l_mp_i = round((self.transitions["indices"][0]+self.transitions["indices"][1])/2)
            r_mp_i = round((self.transitions["indices"][1]+self.transitions["indices"][2])/2)
            l_mp = [self.lrf["angles"][l_mp_i],self.lrf["ranges"][l_mp_i]]
            r_mp = [self.lrf["angles"][r_mp_i],self.lrf["ranges"][r_mp_i]]
        elif self.transitions["n"] == 2:
            m_c = round((self.transitions["indices"][0]+self.transitions["indices"][1])/2)
            l_mp_i = round((self.transitions["indices"][0]+m_c)/2)
            r_mp_i = round((m_c+self.transitions["indices"][1])/2)
            l_mp = [self.lrf["angles"][l_mp_i],self.lrf["ranges"][l_mp_i]]
            r_mp = [self.lrf["angles"][r_mp_i],self.lrf["ranges"][r_mp_i]]
        else:
            self.error["flag"] = True
            self.error["count"] += 1
            r_mp = self.midpoints["r"]
            l_mp = self.midpoints["l"]
        self.midpoints["r"] = r_mp
        self.midpoints["l"] = l_mp
        return

    def main_lgs_tracking(self):
        self.is_oc_done = False
        while not self.rospy.is_shutdown() and not self.is_oc_done:
            self.online_caliration()
        while not self.rospy.is_shutdown():
            if self.change:
                self.count_transitions()
                if self.transitions["n"] == 4 or self.transitions["n"] == 3 or self.transitions["n"] == 2:
                    self.get_midpoints()
                elif self.transitions["n"] >= 6:
                    da = abs(np.diff(transitions["angles"]))
                    indices = np.concatenate((np.nonzero(da<self.legs_params["lp"]),np.nonzero(self.lrf["ranges"][transitions["indices"]+1]==1)))
                    del transitions["angles"][indices]
                    del transitions["ranges"][indices]
                    del transitions["indices"][indices]
                    del transitions["transitions"][indices]
                    self.get_midpoints()
                else:
                    self.get_midpoints()
                self.change = False

if __name__ == '__main__':
    try:
        lt  = LegsTracking()
    except rospy.ROSInterruptException:
        pass
