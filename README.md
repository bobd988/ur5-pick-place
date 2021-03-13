### pick and place in ROS-Gazebo with a USB cam and vacuum grippers. 

<p align="center">
<img src="https://github.com/bobd988/ur5-pick-place/blob/main/src/ur5-pick-place/media/demo1.png" width="600">
<img src="https://github.com/bobd988/ur5-pick-place/blob/main/src/ur5-pick-place/media/demo2.png" width="600">

This project is based on ROS Kinetic under Ubuntu 16.04 LTS.  

### Build
put the projects into a carkin src folder.
```
catkin build 
```

###run the simulation:  

```
roslaunch ur5_notebook initialize.launch
```
This will wait for command for robot arm.

then run ur5_arm.py from command line or Pycharm to direct arm which box robot arm should place. The object_index can be chagned to represent the box index(0-3)




