# Robot controller

## _Assignment no. 3 for  the Research Track 1 course_

### Professor [Carmine Recchiuto](https://github.com/CarmineD8)

The aim of this project is to make a robot move inside a 3D environment. In particular, we were asked to make the robot execute one of the following behaviours upon user request:
- autonomously reach a (x, y) coordinate inserted by the user;
- let the user drive the robot with the keyboard;
- let the user drive the robot assisting them to avoid collisions.

For this assignment, two programs were used: [Gazebo](http://gazebosim.org/) and [Rviz](http://wiki.ros.org/rviz).

The following figure shows the environment on Gazebo used for the simulation.
<p align="center">
<img src= "https://user-images.githubusercontent.com/62473854/160248338-26cc471e-48d3-4129-ba0d-c88a0793a5b3.png" width=70%, height=70%>
</p>

The following figure shows the fully explored environment on Rviz.
<p align="center">
<img src= "https://user-images.githubusercontent.com/62473854/160248847-89a8b20b-55a8-433f-8300-b207c8b21600.png" width=50%, height=50%>
</p>

## Installing and running
The simulation requires the installation of ROS, in particular the Noetic Release of ROS.

Moreover, you need some additional packages; in particular:
- The slam_gmapping package, which can be installed via the following command
```sh
$ git clone https://github.com/CarmineD8/slam_gmapping.git
```
- The ros navigation stack, which can be installed via the following command
```sh
$ sudo apt-get install ros-<your_ros_distro>-navigation
```
- The terminal emulator program XTerm, which can be installed via the following command
```sh
$ sudo apt install xterm
```

After you are sure you have installed all the needed elements, to run the program you need to open a shell window in your ROS workspace and to build the workspace with the command `catkin_make` in the shell; then you must write the following command in the shell window:
```sh
$ roslaunch final_assignment launch.launch
```

## UI node
This node prints on the xterm console a menu which allows the user to chose how to drive the robot. In particular, the user can:
- press 1 to make the robot reach a point autonomously;
- press 2 to drive the robot with the keyboard;
- press r/R to reset the simulation;
- press e/E o exit the program.

Regarding the modality with which the user decides to drive the robot, when the program detects a correct input from the user one of the following two things happens:
1. the user presses 1, so the node relative to the autonomous driving is ran with the command
```cpp
system("rosrun final_assignment auto_controller");
```
2. the user presses 2, so the node relative to the keyboard is ran with the help of a .launch file with the following command
```cpp
system("roslaunch final_assignment keyboard.launch");
```
In the section relative to the Keyboard node, the decision to use a .launch file is explained in detail.
