#!/bin/bash
if [ "$1" == "m" ] || [ "$1" == "mon" ] || [ "$1" == "monitor" ]; then
    # Set the wireless up for monitoring
    sudo ifconfig wlan0 down
    sudo service network-manager stop 
    sudo iwconfig wlan0 mode monitor
    sudo ifconfig wlan0 up
else
    # Let the wireless be handled by the network-manager
    sudo ifconfig wlan0 down
    sudo sudo service network-manager start 
    sudo iwconfig wlan0 mode managed
    sudo ifconfig wlan0 up
fi
