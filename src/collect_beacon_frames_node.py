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

class Beacon:
	def __init__(self, mac, essid, signal, InList):
		self.mac = mac
		self.essid = essid
		self.signal = signal
		self.InList = InList

	def IsInList(self):
		return self.InList

class WifiCollector:
	def __init__(self, interface):
		if interface == "":
			print "Interface not specified"
			exit(1)

		self.lookForMac_list = []

		self.rawSocket = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003))
		self.rawSocket.bind((interface, 0x0003))

	'''
	Look for a specific MAC-Address. If not specified, the class will publish all received beacons
	'''
	def setLookForMac(self, mac):
		self.lookForMac_list.append(mac)

	def getLookForMacList(self):
		# Returns false if the list is empty
		return not self.lookForMac_list

	def getNextBeacon(self):
		pkt = self.rawSocket.recvfrom(2048)[0]
		if pkt[18] == "\x80":
			essid = pkt[56:56 +ord(pkt[55])]
			mac = pkt[28:34].encode('hex')
			signal = 256-int(ord(pkt[14]))
			inList = mac in self.lookForMac_list or not self.lookForMac_list

			tmp = Beacon(mac, essid, signal, inList)
			return tmp

		else:
			return Beacon(0,0,0,False)

wificollector = WifiCollector("mon0")

rospy.init_node('WifiBeaconCollector')
pub_beacon = rospy.Publisher('/WifiBeaconCollector/beacons', BeaconMsg, queue_size=10)

while not rospy.is_shutdown():
	beacon = wificollector.getNextBeacon()
	if beacon.IsInList():
		msg = BeaconMsg()				
		msg.header.stamp = rospy.Time.now()
		msg.essid = beacon.essid
		msg.mac = beacon.mac
		pub_beacon.publish(msg)
