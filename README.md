# DataCamp PCL Tutorial

## Dataset
https://drive.google.com/open?id=1fVjIAJaga9Ulf50WHIp9tVTfLWDxY6f-

## Dependencies:
* ROS Kinetic or Melodic

## How to use:
I will assume you are new to ROS.


```
mkdir -p datacamp_ws/src
cd datacamp_ws/src
git clone https://github.com/xmfcx/pcl-datacamp.git
cd ..
catkin_make
```
This should build the pcldatacamp node in your workspace.

Then you can run ```roscore``` in a terminal. And in another terminal you can run:
```
cd datacamp_ws
source devel/setup.bash
rosrun pcldatacamp pcldatacamp_exec
```
to run the clusterer executable.

To visualize it, in another terminal run ```rviz``` and as a configuration file choose datacamp_ws/src/pcl-datacamp/rviz_stuff/datacamp.rviz

To feed clusterer with a point cloud stream, you can download a bag file from https://github.com/udacity/self-driving-car/tree/master/datasets

and do:
```
rosbag play filename.bag
```
to play it.
