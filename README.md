Autonomous Take-off and Land - ArUco
====================================

### Change Log ###

[Author: Alan Tavares & César Quiroz]

#### Build and tested ###

OS: Ubuntu 16.04 - xenial
ROS version: Kinetic


In order to perform an accurate take-off and land of the Bebop 2, we are using ArUco Markers based on image processing. Firstly, we used a Parrot Bebop 2 model as drone provided by Sphinx-Guide to simulate an environment in Gazebo. The main idea is to use the Aruco Tag to send commands to Bebop 2 to take-off and land autonomously. After simulation, it will be necessary to implement in the real environment using a real Bebop 2.


We use the kinetic installation from: http://wiki.ros.org/kinetic/Installation. Do not forget to install wstool http://wiki.ros.org/wstool

If you are using a Parrot Bebop 1 or 2 model, you'll need to install the ARDroneSDK3 API.

The SDK will help you connect, pilot, receive stream, save and download medias (photo and video), send and play autopilot flight plans and update your drone. You can use it on the Rolling Spider, Cargos, Mambo, Swing, Jumping Sumo, Jumping Sumo Evos, Bebop Drone, Bebop 2, Bebop 2 Power, Disco, Bluegrass, SkyController and SkyController 2.

FreeFlight3 is using this SDK.

This SDK is mainly written is C, it provides libraries for Unix system, Android and iOS.

It also comes with a drone simulator called Sphinx, which is intended to help you test your application before flying with your actual drone. All the information about Sphinx (installation, user manual, application notes) is available HERE (Sphinx-Guide: https://developer.parrot.com/docs/sphinx/installation.html).

## Source: Parrot Bebop Drone ##

You need to install external tools:

* git
* build-essential (only for Linux)
* autoconf
* libtool
* python
* python3
* libavahi-client-dev (only for specific Samples)
* libavcodec-dev (only for specific Samples)
* libavformat-dev (only for specific Samples)
* libswscale-dev (only for specific Samples)
* libncurses5-dev (only for specific Samples)
* mplayer (only for specific Samples)

Download all sources

SDK sources are hosted on github. To download the latest release, you only have to init repo with the arsdk_manifests url:
```
repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git -m release.xml
```
After that, you can download all the other repository automatically by executing the command:
```
repo sync
```

Then follow the How to build the SDK section:

#### Unix Build ####

Linux: Tested on Ubuntu 16.04 - xenial

The command to build the SDK for Unix platform is:

```
./build.sh -p arsdk-native -t build-sdk -j
```
bebop_autonomy - ROS Driver for Parrot Bebop Drone (quadrocopter) 1.0 & 2.0
===========================================================================

bebop_autonomy is a ROS driver for Parrot Bebop 1.0 and 2.0 drones (quadrocopters), based on Parrot’s official ARDroneSDK3. This driver has been developed in Autonomy Lab of Simon Fraser University by Mani Monajjemi and other contributers. This software is maintained by Sepehr MohaimenianPour (AutonomyLab, Simon Fraser University), Thomas Bamford (Dynamic Systems Lab, University of Toronto) and Tobias Naegeli (Advanced Interactive Technologies Lab, ETH Zürich).

Just follow the wiki installation for Parrot 1.0 & 2.0

```
https://bebop-autonomy.readthedocs.io/en/latest/installation.html
```
Start the First Simulation using Bebop 2, Gazebo and Sphinx
===========================================================

Enter in your Catkin Workspace

```
$ cd ~/bebop_ws

$ source devel/setup.bash

$ sudo systemctl start firmwared.service

$ fdc ping

PONG
```

#### Check your wifi interface name ####

```
$ ifconfig
```

Now, Modify the XML file of the bebop2.drone located in:

```
$ cd /opt/parrot-sphinx/usr/share/sphinx/drones/
```

Change the parameter `<stole_interface>` according to your ifconfig result. In our case `wlp3s0`.

```
<?xml version="1.0" encoding="UTF-8"?>
<drone
  name="bebop2"
  firmware="http://plf.parrot.com/sphinx/firmwares/ardrone3/milos_pc/latest/images/ardrone3-milos_pc.ext2.zip"
  hardware="milosboard">
  <machine_params
    low_gpu="false"
    with_front_cam="true"
    with_hd_battery="false"
    with_flir="false"
    flir_pos="tilted"/>
  <pose>default</pose>
  <interface>eth1</interface>
  <!-- 'wlan0' may need to be replaced the actual wifi interface name -->
  <stolen_interface>wlp3s0:eth0:192.168.42.1/24</stolen_interface>
</drone>
```

#### Start simulation ####

```
sphinx /home/cesar/aruco_big_box.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone
```
The world will be executed.

After that, open another terminal and execute the following command in your `catkin workspace`.

```
$ roslaunch bebop_driver bebop_node.launch
```

In another terminal, execute:

```
$ rosrun control_bebop_teleop image_sub
```

In another terminal, execute:

```
$ rosrun control_bebop_teleop landing_pub
```