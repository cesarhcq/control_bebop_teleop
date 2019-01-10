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

```
$ sudo apt install git repo build-essential autoconf libtool python python3 libavahi-client-dev libavcodec-dev libavformat-dev libswscale-dev libncurses5-dev mplayer
```

Download all sources

SDK sources are hosted on github. To download the latest release, you only have to init repo with the arsdk_manifests url:
```
$ repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git -m release.xml
```
After that, you can download all the other repository automatically by executing the command:
```
$ repo sync
```

Then follow the How to build the SDK section:

#### Unix Build ####

Linux: Tested on Ubuntu 16.04 - xenial

The command to build the SDK for Unix platform is:

```
$ ./build.sh -p arsdk-native -t build-sdk -j
```
bebop_autonomy - ROS Driver for Parrot Bebop Drone (quadrocopter) 1.0 & 2.0
===========================================================================

bebop_autonomy is a ROS driver for Parrot Bebop 1.0 and 2.0 drones (quadrocopters), based on Parrot’s official ARDroneSDK3. This driver has been developed in Autonomy Lab of Simon Fraser University by Mani Monajjemi and other contributers. This software is maintained by Sepehr MohaimenianPour (AutonomyLab, Simon Fraser University), Thomas Bamford (Dynamic Systems Lab, University of Toronto) and Tobias Naegeli (Advanced Interactive Technologies Lab, ETH Zürich).

### If you don't have ROS workspace yet you can do so by ###

```
$ mkdir -p ~/bebop_ws/src && cd ~/bebop_ws
$ catkin init
$ git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
# Update rosdep database and install dependencies (including parrot_arsdk)
$ rosdep update
$ rosdep install --from-paths src -i
# Build the workspace
$ catkin build
```

### Get the simulator and additional dependencies ###
```
$ cd ~/bebop_ws/src
$ git clone git@github.com:alanprodam/control_bebop.git
```

```
 bebop_ws
   |
   |---> build
   |---> devel
   |---> logs
   |---> src
	 |
         |---> bebop_autonomy
         |---> control_bebop
```

Just follow the wiki installation for Parrot 1.0 & 2.0

Compiling From Source - [Installation-BebopAutonomy](https://bebop-autonomy.readthedocs.io/en/latest/installation.html)

Sphinx Guide - [Firststep](https://developer.parrot.com/docs/sphinx/firststep.html)


Start the First Simulation using Bebop 2, Gazebo and Sphinx
===========================================================

Enter in your Catkin Workspace

#### Firmwared ####

```
$ sudo systemctl start firmwared.service

```

### Important: Do not forget! You must reboot/start your computer, so the firmwared service will be working. ###

In case you reboot/start your computer, you need to restart the firmwared service.

```
$ sudo firmwared
```

The execution of firmwared is blocking so do not close your shell.

#### Check that firmwared is alive

Enter the following command:

```
$ fdc ping

PONG
```

#### Check your wifi interface name ####

```
$ ifconfig or iwconfig
```

Now, Modify the XML file of the bebop2.drone located in:

```
$ cd /opt/parrot-sphinx/usr/share/sphinx/drones/

$ subl bebop2.drone
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

#### Check the IP for real experiment and simulated experiment  ####

Change the IP `<launch> = bebop_node.launch` according to IP drone. In case of simulation, you should use `default="10.202.0.1"`

```
$ cd /home/<user>/bebop_ws/src/bebop_autonomy/bebop_driver/launch/

$ subl bebop_node.launch
```
```
<?xml version="1.0"?>
<launch>
    <arg name="namespace" default="bebop" />
    <arg name="ip" default="10.202.0.1" /> <!--  <arg name="ip" default="192.168.42.1" | default="10.202.0.1" /> -->
    <arg name="drone_type" default="bebop2" /> <!-- available drone types: bebop1, bebop2 -->
    <arg name="config_file" default="$(find bebop_driver)/config/defaults.yaml" />
    <arg name="camera_info_url" default="package://bebop_driver/data/$(arg drone_type)_camera_calib.yaml" />
    <group ns="$(arg namespace)">
        <node pkg="bebop_driver" name="bebop_driver" type="bebop_driver_node" output="screen">
            <param name="camera_info_url" value="$(arg camera_info_url)" />
            <param name="bebop_ip" value="$(arg ip)" />
            <rosparam command="load" file="$(arg config_file)" />
        </node>
        <include file="$(find bebop_description)/launch/description.launch" />
    </group>
</launch>
```

#### Check calibration file ####

If you need, you can chance of camera calibration file, the first file is bebop2 with 856x480:

```
$ subl /home/<user>/bebop_ws/src/bebop_autonomy/bebop_driver/data/bebop2_camera_calib.yaml
```
```
image_width: 856
image_height: 480
camera_name: bebop_front
camera_matrix:
  rows: 3
  cols: 3
  data: [537.292878, 0.000000, 427.331854, 0.000000, 527.000348, 240.226888, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.004974, -0.000130, -0.001212, 0.002192, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
projection_matrix:
  rows: 3
  cols: 4
  data: [539.403503, 0.000000, 429.275072, 0.000000, 0.000000, 529.838562, 238.941372, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
```

The second file is bebop1 with 640x368:

```
$ subl /home/<user>/bebop_ws/src/bebop_autonomy/bebop_driver/data/bebop1_camera_calib.yaml
```
```
image_width: 640
image_height: 368
camera_name: bebop_front
camera_matrix:
  rows: 3
  cols: 3
  data: [396.17782, 0, 322.453185, 0, 399.798333, 174.243174, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.001983, 0.015844, -0.003171, 0.001506, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [400.182373, 0, 323.081936, 0, 0, 403.197845, 172.320207, 0, 0, 0, 1, 0]
```

#### Start simulation ####

In the first window of terminal:

```
$ sudo systemctl start firmwared.service

$ fdc ping

PONG
```

Check the IP with `iwconfig`:

```
$ cd ~/bebop_ws

$ source devel/setup.bash

$ sphinx /home/<user>/bebop_ws/src/control_bebop/world/aruco_big_box.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone
```

If you wanna find and change the Big_box with ArUco, if you do not have the big_box, move the file to `.gazebo/models/...`:

```
/home/<user>/bebop_ws/src/control_bebop/world/big_box

/home/<user>/.gazebo/models/big_box
```

In case of low GPU, you should put this command
```
$ sphinx /home/<user>/bebop_ws/src/control_bebop/world/aruco_big_box.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone::low_gpu=true

$ sphinx /home/<user>/bebop_ws/src/control_bebop/world/aruco_big_box.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone::with_front_cam=false
```
The world will be executed.

After that, open another terminal and execute the following command in your `catkin workspace`.

```
$ roslaunch bebop_driver bebop_node.launch
```

In another terminal, execute:

```
$ rosrun control_bebop image_sub.py
```

In another terminal, execute:

```
$ rosrun control_bebop landing_pub.py
```

> sphinx /home/victor/bebop_ws/src/control_bebop/world/aruco_big_box.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone


