# Robot controller

## _Assignment no. 3 for  the Research Track 1 course_

### Professor [Carmine Recchiuto](https://github.com/CarmineD8)

The aim of this project is to make a robot move inside a 3D environment. In particular, we were asked to make the robot execute one of the following behaviours upon user request:
- autonomously reach a (x, y) coordinate inserted by the user;
- let the user drive the robot with the keyboard;
- let the user drive the robot assisting them to avoid collisions.

## Installing and running
The simulation requires the installation of ROS, in particular the Noetic Release of ROS.

To run the program, first you need to open a shell window in your ROS workspace and to build the workspace with the command `catkin_make` in the shell; then you must write the following command in the shell window:
```sh
$ roslaunch final_assignment launch.launch
```
