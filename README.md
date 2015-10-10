# WIFIBeaconCollector
Collects beacon frames from avaiable AP's


This is a small node that collects beacons from available access points and publishes them on a topic.

To compile the project, clone it into your working catking_workspace and run catkin_make.

Before starting the node, create mon0 interface using airmon-ng
 sudo aitmon-ng start wlan0

Bring it up
 sudo ifconfig mon0 up


When starting the node, make sure to run it as root.
sudo rosrun probably won't work. Instead write

 sudo -s

Then run the node using rosrun.

If the node is included in a launch-file, it can be run using the same procedure as above (sudo -s then roslaunch )

This way of running the node is not ideal, I still haven't found a better way of doing it.

