from sys  import stdin
import rosbag
import numpy as np
import matplotlib.pyplot as plt


roi = {"angle_min": -30*np.pi/180.0,
            "angle_max": 30*np.pi/180.0,
            "r_max": 1000}
lrf = {"angles": [],
            "ranges": [],
            "transitions": []}

fig = plt.figure()

bag = rosbag.Bag('~/bags/LRF/test.bag')
for _,msg,t in bag.read_messages(topics=['/scan']):
    print(t)
    all_ranges = msg.ranges
    all_angles = np.linspace(msg.angle_min,msg.angle_max,len(all_ranges))
    indices = np.logical_and(all_angles>=roi["angle_min"],all_angles<=roi["angle_max"])
    lrf["angles"] = np.array(all_angles[indices])
    lrf["ranges"] = all_ranges[indices]
    lrf["ranges"][np.nonzero(lrf["ranges"]>roi["r_max"])] = roi["r_max"]
    lrf["diff_ranges"] = abs(np.diff(lrf["ranges"]))

    ax = plt.subplot(1,2,1, projection='polar')
    ax.plot(lrf["angles"]+90*np.pi/180, lrf["ranges"])
    ax.set_rmax(roi["r_max"])
    ax.grid(True)
    ax.set_title("LRF scan")

    ax1 = plt.subplot(1,1,2)
    ax1.plot(lrf["angles"], lrf["diff_ranges"])
    ax1.grid(True)
    ax1.set_title("Transitions")

    plt.show()

    x = stdin.readline()

bag.close()
