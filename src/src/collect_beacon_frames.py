#!/usr/bin/env python
import socket 
import time

lookForMac = ''

rawSocket = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003))
rawSocket.bind(("mon0", 0x0003))

count = 200

while count > 0:
	pkt = rawSocket.recvfrom(2048)[0]
	if pkt[18] == "\x80" :
		essid = pkt[56:56 +ord(pkt[55])]
		mac = pkt[28:34].encode('hex')
		signal = 256-int(ord(pkt[14]))
		print "Time: %s MAC: %s Signal: %d ESSID: %s" % (time.strftime('%H:%M:%S'),mac, signal, essid)
