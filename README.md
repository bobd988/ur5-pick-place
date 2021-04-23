### pick and place in ROS-Gazebo with a USB cam and vacuum grippers. 

<p align="center">
<img src="https://github.com/bobd988/ur5-pick-place/blob/main/src/ur5-pick-place/media/demo1.png" width="600">
<img src="https://github.com/bobd988/ur5-pick-place/blob/main/src/ur5-pick-place/media/demo2.png" width="600">

This project is based on ROS Kinetic under Ubuntu 16.04 LTS.  Make sure you have a working Kinetic environment. Please refer http://wiki.ros.org/kinetic

### Build
Clone the projects into a carkin src folder.
```
catkin build 
```

### run the simulation:  


```
source devel/setup.bash 
roslaunch ur5_notebook initialize.launch
```
This will wait for commands for robot arm to run. Next send command line from a new terminal. The robot will start  pick and place from the box in order (0 to 3) 

```
python ur5_arm.py 
```




