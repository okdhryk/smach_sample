# HSR smach states sample for open source simulator
https://github.com/hsr-project

# Installation
## Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be installed first. Additionally, YOLO for ROS depends on following software:

OpenCV (computer vision library),
boost (c++ library),

## Building

## Basic Usage
```
$ rosrun smach_sample remove_object.py #once delete all objects
$ rosrun smach_sample smach_sample.py 
```
This program repeats five times getting an object (apple) from the shelf and coming back. 
