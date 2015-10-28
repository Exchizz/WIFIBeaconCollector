#!/usr/bin/env python
import socket
import random 
import rospy

'''
Requires root permission to run..
sudo su root
source /opt/ros/setup.bash
source .../frobomind/devel/setup.bash
python collect_beacon_frames_node.py
'''

from WIFIBeaconCollector.msg import Beacon
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# List of collected data
positionList = [] # time,x,y,z
beaconList = [] # time,strength,mac,essi
beaconLists = {} # mac->time,strength,mac,essi

class PosRecord:
    def __init__(self, t_sec, t_nsec, x, y, z):
        self.sec = t_sec
        self.nsec = t_nsec
        self.x = x
        self.y = y
        self.z = z

def processOdometry(odometry):
    # Time
    t_sec = odometry.header.stamp.secs
    t_nsec = odometry.header.stamp.nsecs

    # Position
    x = odometry.pose.pose.position.x
    y = odometry.pose.pose.position.y
    z = odometry.pose.pose.position.z

    # Store position with time
    positionList.append( PosRecord(t_sec, t_nsec, x, y, z) )
    ##rospy.loginfo(rospy.get_caller_id() + " %s", "len(positionList)="+str(len(positionList)))

class BeaconRecord():
    def __init__(self, t_sec, t_nsec, strength, mac, essid):
        self.t_sec = t_sec
        self.t_nsec = t_nsec
        self.strength = strength
        self.mac = mac
        self.essid = essid

    def setPos(self, x, y):
        self.x = x
        self.y = y

def processBeacon(beacon):
    t_sec = beacon.header.stamp.secs
    t_nsec = beacon.header.stamp.nsecs
    bRec = BeaconRecord(t_sec, t_nsec, beacon.signal, beacon.mac, beacon.essid)

    if len(positionList)==0: # empty
        x, y = 0, 0
    else:
        x, y = positionList[-1].x, positionList[-1].y
    bRec.setPos(x, y)
    beaconList.append(bRec)

    if beacon.mac in beaconLists:
        beaconLists[beacon.mac].append(bRec)
    else:
        beaconLists[beacon.mac] = [bRec]

    #rospy.loginfo(rospy.get_caller_id() + " %s", beaconLists.keys())


def listener():
    rospy.Subscriber("/fmKnowledge/pose", Odometry, processOdometry) # type nav_msgs/Odometry
    rospy.Subscriber("/WifiBeaconCollector/beacons", Beacon, processBeacon) # type nav_msgs/Odometry

def publishRoute(publisher):
    #publish marker
    points = Marker()
    points.header.frame_id = "/map";
    points.header.stamp = rospy.Time.now()
    points.ns = "points_and_lines";
    points.action = Marker.ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;

    points.type = Marker.POINTS;

    # POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    # Points are green
    points.color.g = 1.0;
    points.color.a = 1.0;

    # Create the vertices for the points and lines
    for r in positionList:
        p = Point();
        p.x = r.x
        p.y = r.y
        p.z = r.z
        points.points.append(p)

    # write to topic
    publisher.publish(points)

colorMap = {}
def getColorForMac(mac):
    if mac not in colorMap:
        r = random.uniform(0,1)
        b = random.uniform(0,1)
        g = random.uniform(0,1)
        colorMap[mac]=ColorRGBA(r, g, b, 1.0)
    return colorMap[mac]

def publishSignalStrength(publisher):
    for mac in beaconLists:
        l = beaconLists[mac]
        #publish markers
        points = Marker()
        points.header.frame_id = "/map";
        points.header.stamp = rospy.Time.now()
        points.ns = "points_and_lines";
        points.action = Marker.ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;

        points.type = Marker.POINTS;

        # POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        # Points color
        points.color = getColorForMac(mac)

        # Create the vertices for the points and lines
        for b in l:
            p = Point();
            p.x = b.x
            p.y = b.y
            p.z = b.strength
            points.points.append(p)

        # write to topic
        publisher.publish(points)


def talker():
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        publishRoute(rospy.Publisher("route", Marker, queue_size=10))
        publishSignalStrength(rospy.Publisher("signal", Marker, queue_size=100))
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fuser')
    listener()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
