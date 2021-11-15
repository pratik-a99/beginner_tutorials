# beginner_tutorials
[![GitHub license](https://badgen.net/github/license/Naereen/Strapdown.js)](LICENSE.md)

This repository contains a ROS C++ publisher(talker.cpp) that can count till infinity(if possible) and a ROS C++ subscriber(listener.cpp) that can listen to the talker until the end of time. The talker has a service that can be used to change the published message of the talker. Two launch files are provided, which can take arguments and open both nodes simultaneously. Tf is used to demonstrate tranfrom functions. Rostests are also used to demonstrate Level 2 testing framework. Rosbag has also been used to record the topics.

## Dependencies
* ROS (Melodic preferred)
* xTerm (Optional)

## Usage

### Downloading the repository

First clone the repository
```
git clone https://github.com/pratik-a99/beginner_tutorials.git
```

### Adding it to ROS workspace
Then, add the downloaded repository into your ROS workspace's src folder (eg. catkin_make/src)
```
cd ~/carkin_ws/src
```
To build the changes use
```
cd ..
catkin_make
```

### Executing the code
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

#### Launch files

To use the launch files, there are two options available. 
1. Open both the nodes in the same terminal. The count_rate argument can be changed.
```
roslaunch beginner_tutorials launchfile.launch count_rate:=1
```
2. Open nodes in their separate terminals using xterm. This requires `xterm` installed in the ubuntu environment. \
To install xterm, use 
```
sudo apt install xterm
```
After installation, use the following launch file as given below
```
roslaunch beginner_tutorials launchfile_xterm.launch count_rate:=10
```
#### Service
Use the following command, after starting the nodes, to run call the service. The string can be changed to a string of your choice.
```
rosservice call /outputService "Service is working"
```
#### TF 
Tf has been added to the talker which can be run using the above mentioned launch files command and can be inspected as below(in a new terminal) : 
```
rosrun tf tf_echo world talk
```
The above command will output the transformed frames on the console.

To view the rqt tree of the tf : 
```
rosrun rqt_tf_tree rqt_tf_tree
```
#### Rostest
Rostest has been added to the repository, which can be checked via the following command line

```
rostest beginner_tutorials ros_test.launch 
```
#### Rosbag
To enable rosbag recording, the `record` argument can be set to `true` while using the above-mentioned launch files. `record` is set to `false` while in default, hence no recording will happen until it's set `true`. The recorded rosbags will be stored in the results folder. For example : 
```
roslaunch beginner_tutorials launchfile_xterm.launch count_rate:=1 record:=true
```
or
```
roslaunch beginner_tutorials launchfile.launch count_rate:=1 record:=true
```

The recorded rosbag file can be inspected by using : 
```
rosbag info recording.bag 
```

To replay the rosbags, first start roscore, and the listener node in two separate terminal windows as below
```
roscore
```
```
rosrun beginner_tutorials listener 
```
And then replay then play the recordings from the recorded rosbag, which will be verified when the listener node starts showing the output stream.
```
rosbag play recording.bag
```

### Using Releases
If the the repository is downloaded as a zip via `Week11_HW_Release`, extract the file to `<your_ros_workspace>/src` (eg. `catkin_ws/src`) and rename the folder to `beginner_tutorials`
