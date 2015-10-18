# WIFIBeaconCollector
Collects beacon frames from avaiable AP's


This is a small node that collects beacons from available access points and publishes them on a topic.

To compile the project, clone it into your working catking_workspace and run catkin_make.

Before starting the node, put your wireless lan card in monitor mode:
```
$ sudo ifconfig wlan0 down
$ sudo service network-manager stop # When running Ubuntu
$ sudo iwconfig wlan0 mode monitor
$ sudo ifconfig wlan0 up
```

When starting the node, make sure to run it as root.
$ sudo rosrun probably won't work. Instead use
```
$ sudo -s
```
Then run the node using rosrun.

If the node is included in a launch-file, it can be run using the same procedure as above (sudo -s then roslaunch ... )

This way of running the node is not ideal, I still haven't found a better way of doing it.


Afterwards, let the network manager manage the wireless lan card:
$ sudo ifconfig wlan0 down
$ sudo sudo service network-manager start 
$ sudo iwconfig wlan0 mode managed
$ sudo ifconfig wlan0 up


To change the frequency to monitor:
$ sudo iwconfig wlan0 channel <1,2,..,N>

For example:
$ sudo iwconfig wlan0 channel 2

Here N is the number of possible to channels which can be listed using iwlist.


