# beginner_tutorials
[![GitHub license](https://badgen.net/github/license/Naereen/Strapdown.js)](LICENSE.md)

This repository contains a ROS C++ publisher(talker.cpp) that can count till infinity(if possible) and a ROS C++ subscriber(listener.cpp) that can listen to the talker until the end of time.

## Dependencies
* ROS (Melodic preferred)

## Usage

First clone the repositiory
```
git clone https://github.com/pratik-a99/beginner_tutorials.git
```
Then, add the downloaded repository into your ROS workspace's src folder (eg. catkin_make/src)
```
cd ~/carkin_ws/src
```
To build changes use
```
cd ..
catkin_make
```
Run the following three commands in three different terminals 
```
roscore
```
```
rosrun beginner_tutorials talker 
```
```
rosrun beginner_tutorials listener
```
