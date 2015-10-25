#!/usr/bin/env python
import socket 
import rospy

'''
Requires root permission to run..
sudo su root
source /opt/ros/setup.bash
source .../frobomind/devel/setup.bash
python collect_beacon_frames_node.py
'''

from WIFIBeaconCollector.msg import Beacon as BeaconMsg
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# List of collected data
positionList = [] #time,x,y,z

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

def listener():
    rospy.Subscriber("/fmKnowledge/pose", Odometry, processOdometry)#type nav_msgs/Odometry

def talker():
    rate = rospy.Rate(10) # Hz
    publisher = rospy.Publisher("points", Marker, queue_size=10)
    while not rospy.is_shutdown():
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

        publisher.publish(points)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fuser')
    listener()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
