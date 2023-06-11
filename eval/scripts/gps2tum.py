
import rosbag
import rospy
import jsoncomment

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

import numpy as np
from math import sin, cos, sqrt, pow
from math import pi as PI
import tf.transformations as tts

class GPSTool:

    def __init__(self):

        self.lla_ori = np.array([34.0, 113.0, 72.0])
        self.to_rad = 0.01745329252
        self.earth_major = 6378137.0		 # ///< WGS84 MAJOR AXIS
        self.earth_minor = 6356752.31424518  # ///< WGS84 MINOR AXIS

    def lla2ecef(self, lla):
        ecef = [0] * 3
        lat = lla[0] * self.to_rad
        lon = lla[1] * self.to_rad
        alt = lla[2]
        earth_r = pow(self.earth_major, 2) / sqrt(pow(self.earth_major * cos(lat), 2) + pow(self.earth_minor * sin(lat), 2))
        ecef[0] = (earth_r + alt) * cos(lat) * cos(lon)
        ecef[1] = (earth_r + alt) * cos(lat) * sin(lon)
        ecef[2] = (pow(self.earth_minor / self.earth_major, 2) * earth_r + alt) * sin(lat)
        return ecef
    
    def ecef2enu(self, ecef):
        lat = self.to_rad * self.lla_ori[0]
        lon = self.to_rad * self.lla_ori[1]     
        t = -np.array(self.lla2ecef(self.lla_ori))
        r = np.array([-sin(lon), cos(lon), 0, 
			-cos(lon) * sin(lat), -sin(lat) * sin(lon), cos(lat),
			cos(lon) * cos(lat), sin(lon) * cos(lat), sin(lat)]).reshape((3,3))
        
        enu = np.array(ecef) + t
        enu = np.matmul(r, enu)
        return enu
    
    def getEnu(self, lla):
        return self.ecef2enu(self.lla2ecef(lla))


# cfg = None
# with open('../../config/params.json', 'r') as f:
#     cfg = jsoncomment.JsonComment().load(f)
#     print(cfg)

gga_topic = "/gnss/gga"
pos_topic = "/novatel718d/pos"

topics = [gga_topic, pos_topic]

gga = []
pos_time = []
pos = []

# with rosbag.Bag(cfg["rosbag"]) as bag:
with rosbag.Bag("/media/gy/201E-43FF/binhai_gps0711-0706.bag") as bag:

    for topic, msg, t in bag.read_messages(topics=topics):

        ft = rospy.Time.to_sec(t)

        if topic == gga_topic:
            stat = msg.data.split(',')[6]
            gga.append(int(stat))
        elif topic == pos_topic:
            # msg : NavSatFix
            pos_time.append(ft)
            pos.append([msg.latitude, msg.longitude, msg.altitude])

        if len(gga) % 50 == 0:
            print(1.0 * len(gga) / 40205)

gps = GPSTool()

with open('../data/bh/gps_tum.txt', 'w') as f:
    for i in range(len(gga)):
        if gga[i] == 4:
            enu = gps.getEnu(pos[i])   
            f.write("{} {} {} 0 0 0 0 1.0\n".format(pos_time[i], enu[0], enu[1]))
