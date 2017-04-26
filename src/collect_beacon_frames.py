#!/usr/bin/env python
import socket
import time

lookForMac = ''

rawSocket = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003))
rawSocket.bind(("wlan0", 0x0003))

count = 200

# The following indices is found by inspection of wlan traffic in Wireshark.
while count > 0:
        pkt = rawSocket.recvfrom(2048)[0]
        if pkt[18] == "\x80" :
                essid = pkt[56:56 + ord(pkt[55])]
                mac = pkt[28:34].encode('hex')
                signal = 256-int(ord(pkt[14]))
                freq = int(pkt[11].encode('hex') + pkt[10].encode('hex'),16)
                channel = 1 + (freq-2412) / 5 # substract 2412(channel 1), divide by 5 since it's the distance between center freqs
                print "Time: %s MAC: %s Signal: %d ESSID: %s, Freq: %d, Channel: %d" % (time.strftime('%H:%M:%S'),mac, signal, essid, freq, channel)
