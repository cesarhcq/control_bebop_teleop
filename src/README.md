Autonomous Take-off and Land - ArUco and SVO
====================================

### Change Log ###

[Author: Alan Tavares & César Quiroz]

#### Build and tested ###

OS: Ubuntu 16.04 - xenial
ROS version: Kinetic

### Get the simulator and additional dependencies ###

```
$ sudo apt-get install ros-kinetic-cmake-modules libboost-all-dev 

$ cd ~/bebop_ws/src

$ git clone https://github.com/strasdat/Sophus.git

$ git clone https://github.com/uzh-rpg/fast.git

$ git clone https://github.com/uzh-rpg/rpg_vikit.git

$ git clone https://github.com/uzh-rpg/rpg_svo.git

$ cd ~/bebop_ws

$ catkin build
```

Only required if you want to run bundle adjustment. It is not necessary for visual odometry. In fact, we don't run it on our MAVs. g2o requires the following system dependencies: `libeigen3-dev, libsuitesparse-dev, libqt4-dev, qt4-qmake, libqglviewer-dev-qt4`, install them with `apt-get install`

```
$ sudo apt-get install libeigen3-dev libsuitesparse-dev libqt4-dev qt4-qmake libqglviewer-dev-qt4

$ cd ~/bebop_ws/src

$ wget https://github.com/RainerKuemmerle/g2o/archive/20160424_git.tar.gz -O g2o-20160424_git.tar.gz
tar xvzf g2o-20160424_git.tar.gz
```

For more details, follow the wiki installation:

Semi-direct Visual Odometry - [ROS-SVO](https://github.com/uzh-rpg/rpg_svo)

Packages for SVO - [Installation-SVO](https://github.com/uzh-rpg/rpg_svo/wiki/Installation:-ROS)

Obtaining Best Performance - [Best Performance SVO](https://github.com/uzh-rpg/rpg_svo/wiki/Obtaining-Best-Performance)

```
 bebop_ws
   |
   |---> build
   |---> devel
   |---> logs
   |---> src
   |
         |---> bebop_autonomy
         |---> control_bebop_teleop
         **New Packages**
         |---> Sophus
         |---> rpg_vikit
         |---> fast
         |---> g2o
         |---> rpg_svo
```

Start the First Simulation using Bebop 2, Gazebo, Sphinx and SVO
=================================================================

#### Check the Camera Calibration ####

Change the calibration data in `camera_pinhole.yaml` according to our camera.

```
$ subl /home/victor/bebop_ws/src/rpg_svo/svo_ros/param/camera_pinhole.yaml
```
```
  cam_model: Pinhole
  cam_width: 856
  cam_height: 480
  cam_fx: 537.292878
  cam_fy: 427.331854
  cam_cx: 527.000348
  cam_cy: 240.226888
  cam_d0: 0.004974
  cam_d1: -0.000130
  cam_d2: -0.001212
  cam_d3: 0.002192
```

[Calibration-SVO](https://github.com/uzh-rpg/rpg_svo/wiki/Camera-Calibration)


#### Change the lauch for simulated experiment ####

Open the `live.launch` and change the `cam_topic` with `value="/bebop/image_raw"`:

```
$ subl /home/<user>/bebop_ws/src/rpg_svo/svo_ros/launch/live.launch
```

You can choose between `camera_pinhole.yaml` or `camera_atan.yaml` and `vo_fast.yaml` or `vo_accurate.yaml`:

```
<launch>
  
    <node pkg="svo_ros" type="vo" name="svo" clear_params="true" output="screen">
    
        <!-- Camera topic to subscribe to -->
        <param name="cam_topic" value="/bebop/image_raw" type="str" />
        
        <!-- Camera calibration file -->
        <rosparam file="$(find svo_ros)/param/camera_pinhole.yaml" /> <!-- "$(find svo_ros)/param/camera_atan.yaml" | "$(find svo_ros)/param/camera_pinhole.yaml" -->
        
        <!-- Default parameter settings: choose between vo_fast and vo_accurate -->
        <rosparam file="$(find svo_ros)/param/vo_fast.yaml" />

    </node>
        
</launch>
```

Start Simulation with SVO
=================================================================

Open a new console and start SVO with the prepared launchfile:

```
$ cd bebop_ws/

$ source devel/setup.bash

$ roslaunch svo_ros live.launch
```

```
$ rosrun control_bebop_teleop svo_sub.py
```

Start RVIZ (Robot Visualizer) in a new console:

```
$ rosrun rviz rviz -d <PATH TO rpg_svo>/svo_ros/rviz_config.rviz
```

If you wanna open with dataset. Open a new console and change to the directory where you have downloaded the example dataset. Then type:

```
$ rosbag play <name-of-dataset>.bag
```

Now you should see the video with tracked features (green) and in RViz how the camera moves. If you want to see the number of tracked features, fps and tracking quality, run the GUI.

