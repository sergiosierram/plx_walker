from USB6009_v2 import USB6009
import rospy

class DAQ_v2(object):
    def __init__(self):
        self.NIDevice = USB6009()
        #self.NIDevice.ini()
        #self.NIDevice.reset()
        rospy.loginfo("Connecting to USB6009")
        ## C
        self.m = 22.95
        self.b = 0.25
        ## C
        self.channels = (3,2,7,6)

    def get_forces(self):
        channels_info = self.NIDevice.readchannel(self.channels)
        ry_force = channels_info[0][0]*self.m + self.b
        rz_force = channels_info[0][1]*self.m + self.b
        lz_force = channels_info[0][2]*self.m + self.b
        ly_force = channels_info[0][3]*self.m + self.b
        #print(channels_info)
        return ("{0:.3f}\t{1:.3f}\t{2:.3f}\t{3:.3f}".format(ly_force, lz_force, ry_force, rz_force))

    def close_port(self):
        try:
        	self.NIDevice.cerrar()
        	rospy.loginfo("Closing of USB6009 succeed")
        except:
        	rospy.loginfo("Closing of USB6009 failed")
        return
