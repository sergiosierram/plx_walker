## -- Libraries  -- Libraries  -- Libraries  -- Libraries  -- Libraries  -- Libraries   
from __future__ import division

import serial
import hokuyo
import serial_port
import time
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import threading
import math

import time

import random
from sklearn.cluster import KMeans

## -- Function Polar to cartesian -- Function Polar to cartesian

def polar_to_cartesian(radius_list,angles_list):
    cordenates_XY = []
    X_codenates = []
    Y_codenates = []
    for x in range (0,len(radius_list)):
        X = np.cos(angles_list[x]*np.pi/180)*radius_list[x]
        Y = np.sin(angles_list[x]*np.pi/180)*radius_list[x]
        cordenates_XY.append([X,Y])
        X_codenates.append(X)
        Y_codenates.append(Y)
    return cordenates_XY,X_codenates,Y_codenates

## Find cluster legs

def Cluster_legs():

    global Laser_lec, Clusters_legs, prev_cluster, LDD_LRF, Clusters_left_leg,Clusters_rigth_leg


    LRF_distances = []
    LRF_Angles = []

    for x in range(0,len(Laser_lec)):
        LRF_distances.append(Laser_lec[x][1])
        LRF_Angles.append(Laser_lec[x][0])

    Transitions_position = []
    Threshold = 200
    for x in range (1,len(LRF_distances)):
        b = LRF_distances[x] - LRF_distances[x-1]
        if ((b > Threshold) or (b < (-1*Threshold))):
            Transitions_position.append(x)
            
    if len(Transitions_position) == 4 and len(prev_cluster) == 0:
        points,X_p,Y_p = polar_to_cartesian(LRF_distances,LRF_Angles)
        kmeans = KMeans(n_clusters=3, random_state=0).fit(points)
        prev_cluster = []
        for y in range (0,len(kmeans.cluster_centers_)):
            magnitude = (kmeans.cluster_centers_[y][0]**2 + kmeans.cluster_centers_[y][1]**2)**(0.5)
            if magnitude > 100:
                prev_cluster.append([kmeans.cluster_centers_[y][0],kmeans.cluster_centers_[y][1]])
                
        if prev_cluster[0][0] > prev_cluster[1][0]:
            Clusters_rigth_leg = prev_cluster[0]
            Clusters_left_leg = prev_cluster[1]
        else:
            Clusters_rigth_leg = prev_cluster[1]
            Clusters_left_leg = prev_cluster[0]
            
        LDD_LRF.append(Clusters_rigth_leg[1] - Clusters_left_leg[1])
        return True
        
                
        
    elif len(prev_cluster) == 2:
        points,X_p,Y_p = polar_to_cartesian(LRF_distances,LRF_Angles)
        Threshold_cluster = 400
        new_points = []
        Threshold_mag_clster = 30

        for y in range (0,len(points)):
            distance_1 = ( (prev_cluster[0][0] - points[y][0])**2 + (prev_cluster[0][1] - points[y][1])**2 )**(0.5) 
            distance_2 = ( (prev_cluster[1][0] - points[y][0])**2 + (prev_cluster[1][1] - points[y][1])**2 )**(0.5)           
            if distance_2 < Threshold_cluster or distance_1 < Threshold_cluster:
                new_points.append(points[y])
        
        mag_cluster_1 = (prev_cluster[0][0]**2 + prev_cluster[0][1]**2)**(0.5)
        mag_cluster_2 = (prev_cluster[1][0]**2 + prev_cluster[1][1]**2)**(0.5)
        if len(new_points) > 1 and (mag_cluster_1 > Threshold_mag_clster) and (mag_cluster_2 > Threshold_mag_clster):
            kmeans = KMeans(n_clusters=2, random_state=0).fit(new_points)
            prev_cluster = []
            for z in range (0,len(kmeans.cluster_centers_)):
                prev_cluster.append([kmeans.cluster_centers_[z][0],kmeans.cluster_centers_[z][1]])

            if prev_cluster[0][0] > prev_cluster[1][0]:
                Clusters_rigth_leg = prev_cluster[0]
                Clusters_left_leg = prev_cluster[1]
            else:
                Clusters_rigth_leg = prev_cluster[1]
                Clusters_left_leg = prev_cluster[0]
            
            LDD_LRF.append(Clusters_rigth_leg[1] - Clusters_left_leg[1])
            return True
            
        else:
            prev_cluster = []
            return False
            
    else:
        prev_cluster = []
        return False
        
    

## -- Laser range finder scan object -- Laser range finder scan object -- Laser range finder scan object

class LRF(object):
    def __init__(self,uart_speed = 9800,uart_port = "COM1",star_step = 44,stop_step = 725):
      
        self.laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        self.port = serial_port.SerialPort(self.laser_serial)
        self.laser = hokuyo.Hokuyo(self.port)
        self.star_step = star_step
        self.stop_step = stop_step

    def laser_on(self):
        self.laser.laser_on()

    def laser_off(self):
        self.laser.laser_off()
        
    def read_one_scan(self):
        return self.laser.get_single_scan(self.star_step, self.stop_step, 1)

## -- Port LRF detection -- Port LRF detection -- Port LRF detection -- Port LRF detection 

def Find_port_URG_laser():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    Found = False 
    for x in ports:
        if x[1].find("URG Series") == 0:
            Laser_port = x[0]
            Found = True
            break

    if Found == True:
        return Laser_port
    else:
        print ("didn't find laser port")
        return "Error"


## -- Animate plot fuction -- Animate plot fuction -- Animate plot fuction -- Animate plot fuction
    
def animate(i):

    global Laser_lec,prev_cluster,LDD_LRF,W_WFLC_plot,Cadence_WFLC_LRF

    length_to_plot = 200
    
    #theta = np.array([x[0]*np.pi/180 for x in Laser_lec])
    #radius = np.array([x[1] for x in Laser_lec])

    LRF_distances1 = []
    LRF_Angles1 = []

    for x in range(0,len(Laser_lec)):
        LRF_distances1.append(Laser_lec[x][1])
        LRF_Angles1.append(Laser_lec[x][0])

    points1,X_p1,Y_p1 = polar_to_cartesian(LRF_distances1,LRF_Angles1)
    
    ax0.clear()
    ax0.set_ylim(0,2000)
    ax0.set_xlim(-1000,1000)
    ax0.grid(True)
    ax0.set_ylabel("(mm)")
    ax0.set_title("Legs detection")
    
    
    #ax.plot(theta, radius, color='b', linewidth=1)
    ax0.plot(X_p1, Y_p1, color='b', linewidth=1)

    if len(prev_cluster) == 2:
        theta1 = []
        radius1 = []
        
        for m in range(0,len(prev_cluster)):
            theta1.append(np.arctan(prev_cluster[m][1]/prev_cluster[m][0])*-180/np.pi)
            radius1.append( (prev_cluster[m][1]**2 + prev_cluster[m][0]**2)**(0.5) )
        #ax.plot(theta1, radius1, color='k', linewidth=1)
       
        ax0.plot(Clusters_left_leg[0], Clusters_left_leg[1],'ko', linewidth=3)
        ax0.plot(Clusters_rigth_leg[0], Clusters_rigth_leg[1],'ro', linewidth=2)   
    #ax.set_rmax(2000)
    

    if len(LDD_LRF) > length_to_plot:
        for x in range(len(LDD_LRF) - length_to_plot,len(LDD_LRF)):
            LDD_to_show = LDD_LRF[len(LDD_LRF) - length_to_plot:len(LDD_LRF)]
            W_WFLC_show = W_WFLC_plot[len(W_WFLC_plot) - length_to_plot:len(W_WFLC_plot)]
            Cadence_WFLC_LRF_plot = Cadence_WFLC_LRF[len(Cadence_WFLC_LRF) - length_to_plot:len(Cadence_WFLC_LRF)]
    else:
        LDD_to_show = LDD_LRF
        W_WFLC_show = W_WFLC_plot
        Cadence_WFLC_LRF_plot = Cadence_WFLC_LRF        

    ax1.clear()
    ax1.set_title("LDD signal")
    ax1.set_ylabel("(mm)")
    ax1.set_ylim(-700,700)
    ax1.grid(True)
    ax1.plot(LDD_to_show)

    ax2.clear()
    ax2.set_title("FLC Amplitude")
    ax2.set_ylabel("(mm)")
    ax2.set_ylim(0,1000)
    ax2.plot(W_WFLC_show)
    ax2.grid(True)
    
    ax3.clear()
    ax3.set_title("WFLC Cadence")
    ax3.set_ylabel("(Hz)")
    ax3.plot(Cadence_WFLC_LRF_plot)
    ax3.set_ylim(0,1.5)
    ax3.grid(True)

    if len(prev_cluster) == 2:
        Estimate_amplitude =   W_WFLC_show[len(W_WFLC_show)-1]*((0.3142*Cadence_WFLC_LRF_plot[len(Cadence_WFLC_LRF_plot)-1]) + 1.0283)
        #vel1 = (Cadence_WFLC_LRF_plot[len(Cadence_WFLC_LRF_plot)-1] *2* W_WFLC_show[len(W_WFLC_show)-1]/1000 *60*60)/1000
        vel1 = (Cadence_WFLC_LRF_plot[len(Cadence_WFLC_LRF_plot)-1] *2 * Estimate_amplitude/1000 *60*60)/1000
        #print("Speed: " +  str(vel1) + "Km/h")
        ax0.text(-700, 1700, "Speed gate: " + str(vel1) + "Km/h")
        
    
## -- Thread for LRF Lecture

def LRF_Lecture():
    
    global Laser_lecture,Laser_obj,Laser_lec,Time_LRF,LDD_LRF

    while Laser_lecture:
        Laser_lec = Laser_obj.read_one_scan()
        Time_LRF.append(time.time())
        new_lec = []
        for x in range(0,len(Laser_lec)):
            if Laser_lec[x][1] > 2000:
                new_lec.append((Laser_lec[x][0],0))
            else:
                new_lec.append((Laser_lec[x][0],Laser_lec[x][1]))
        Laser_lec = new_lec
                
        Laser_scan_regitry.append(Laser_lec)

        legs_clust_detect = Cluster_legs()
        
        if legs_clust_detect:
            wflc (LDD_LRF[len(LDD_LRF)-1]/600, 0.07, 0.1, 0, 1)

    print("Laser off")

## -- Digital filter WFLC -- Digital filter WFLC -- Digital filter WFLC
    
def wflc (senal, mu_0, mu_1, mu_b, M):

    global sum_omega_0,W_WLFC,Cadence_WFLC_LRF,omega_0
    
    T =1/10
    X=[]
     
    for j in range (0,M):
        X.append(math.sin((j+1)*sum_omega_0))
        X.append(math.cos((j+1)*sum_omega_0))

    MUL_XW=0
    for z in range (0,len(W_WLFC)):
        MUL_XW= MUL_XW + X[z]* W_WLFC[z]
        
    error = senal - MUL_XW - mu_b


    sum_rec = 0
    for j in range (0,M):  
        sum_rec = sum_rec + (j+1)*(W_WLFC[j]*X[M+j] - W_WLFC[M+j]*X[j])

    omega_0_pred = abs(omega_0 + 2*mu_0*error*sum_rec)

    W_pred = []
    for z in range(0,len(W_WLFC)):
        W_pred.append(W_WLFC[z] + 2*mu_1*X[z]*error)
     
    #omega_0.append(omega_0_pred)
    
    
    W_WLFC=[]
    for z in range(0,len(W_pred)):
        W_WLFC.append(W_pred[z])
                
    sum_omega_0    = sum_omega_0 + omega_0_pred;
    #sum_pend = abs(((sum_omega_0 - sum_omega_0_ant))*57.29578)
        

    harmonics = []
    for z in range(0,len(W_WLFC)):
        harmonics.append(W_WLFC[z] * X[z])

    
    suma = 0
    for z in range(0,len(harmonics)):    
        suma =  suma + harmonics[z]

    tremor = suma
 
    omega_0_Hz = (omega_0_pred/(2*np.pi)/T)
    omega_0 = omega_0_pred
    Cadence_WFLC_LRF.append(omega_0_Hz)
    W_WFLC_plot.append( (W_WLFC[0]**2 + W_WLFC[1]**2)**(0.5)*600 )

    Reg_cadence.append(omega_0_Hz)
    Reg_Amplit.append((W_WLFC[0]**2 + W_WLFC[1]**2)**(0.5)*600 )
    Reg_LDD.append(senal*600)

   


# -- Global variables -- Global variables -- Global variables -- Global variables

Laser_obj = None # Object to manipulate the Hokuyo URG scan laser
Laser_lec = [] # List that contains the laser's samples
Laser_lecture = False # Variable for begin or finish the lecture of the LRF
Time_LRF = [] # List that contains the time of each sample of the LRF
Laser_scan_regitry = [] # List that each list scan

Clusters_left_leg = []
Clusters_rigth_leg = []
prev_cluster = []
LDD_LRF = []

Cadence_WFLC_LRF = []
sum_omega_0 = 1.7
W_WLFC = [0,0]
omega_0 = 0.05*2*np.pi*(1/10)

W_WFLC_plot = []

Reg_cadence = []
Reg_Amplit = []
Reg_speed = []
Reg_LDD = []




## -- Program's Main -- Program's Main -- Program's Main -- Program's Main -- Program's Main
   
if __name__ == "__main__":
    
    LRF_port = Find_port_URG_laser()
    start = 300
    stop = 468
    serial_comunication_speed = 256000

    if LRF_port != "Error":

        #global Laser

        # LRF object declaration
        
        Laser_obj = LRF(serial_comunication_speed,LRF_port,start,stop)
        Laser_obj.laser_on()

        # Thead LRF lecture begins
        
        Laser_lecture = True
        try:
            thread_LRF_lecture = threading.Thread(target=LRF_Lecture)
            thread_LRF_lecture.daemon = True
            thread_LRF_lecture.start()

        except:
            Laser_obj.laser_off()
            print("thread of the LRF lecture does not work")
        
        # Initialize the figure to plot
        
        fig = plt.figure()
        #ax = fig.add_subplot(111, projection='polar')
        ax0 = fig.add_subplot(2,2,1)
        ax1 = fig.add_subplot(2,2,2)
        ax2 = fig.add_subplot(2,2,3)
        ax3 = fig.add_subplot(2,2,4)
        
        ani= animation.FuncAnimation(fig, animate, interval=1)

        fig.show()
        Laser_lecture = False
        Laser_obj.laser_off()
        
        Data_file = open('Tets.txt', 'a+')
        for x in range (len(Reg_cadence)):
            Data_file.write(str(Reg_cadence[x]) + ',' + str(Reg_Amplit[x]) + ',' + str(Reg_LDD[x]) + '\n')
            
        Data_file.close()


        
