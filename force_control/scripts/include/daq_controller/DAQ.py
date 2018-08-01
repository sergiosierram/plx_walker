from USB6009 import USB6009
import time

class DAQ(object):
    def __init__(self):
        self.NIDevice = USB6009()
        self.NIDevice.ini()
        self.NIDevice.Reset()
        print("Connecting to USB6009")

        self.mrxy = 250.0/4.5
        self.bry = 10+8.0/9.0
        self.mrz = -200.0
        self.brz = -31.7
        
        self.mlxy = -25.0
        self.bly = 2.2
        self.mlz = 50.0
        self.blz = -4


    def get_forces(self,x_forces = False):
        rx_force = self.NIDevice.leerS0()*self.mrxy - (self.mrxy*0.191)
        ry_force = self.NIDevice.leerS4()*self.mrxy - self.bry
        rz_force = self.NIDevice.leerS1()*self.mrz - self.brz

        lx_force = self.NIDevice.leerS5()*self.mlxy
        ly_force = self.NIDevice.leerS2()*self.mlxy - self.bly
        lz_force = self.NIDevice.leerS6()*self.mlz - self.blz

        #ry_force /= rz_force
        #ly_force /= lz_force

        if not x_forces:
            return("{0:.3f}\t{1:.3f}\t{2:.3f}\t{3:.3f}".format(ry_force,rz_force,ly_force,lz_force))
        else:
            return("{0:.3f}\t{1:.3f}\t{2:.3f}\t{3:.3f}\t{4:.3f}\t{5:.3f}".format(rx_force,ry_force,rz_force,lx_force,ly_force,lz_force))

    def close_port(self):
        print ("Closing ...")
        # NIDevice.close()

if __name__ == '__main__':
	DAQ_session = DAQ()
	try:
		print("Acquiring")
		while True:
			ry,rz,ly,lz = DAQ_session.get_forces(False).split('\t')
			print("Rigth Sensor:")
			print("y: {0}".format(ry))
			print("z: {0}".format(rz))
			print("Left Sensor:")
			print("y: {0}".format(ly))
			print("z: {0}".format(lz))
		 	time.sleep(0.5)

	except KeyboardInterrupt:
	        print ("\nQuitting ...")

	finally:
        	DAQ_session.close_port()
