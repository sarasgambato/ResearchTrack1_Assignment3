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
The little black square in the map represents the robot, which has 720 sensors distributed in a range of 180°, from -90° to 90°, useful to detect whether there is an obstacle near the robot.

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

If the user presses r/R then the robot is spawned to its initial position and the velocity is reset via the service `gazebo/reset_simulation`.

## ReachPoint node
This node lets the user choose a point (x, y) that the robot will try to reach on its own.

Firstly, the node prints a menu:
- press p/P to set the coordinates of the goal;
- press c/C to cancel the current goal;
- press b/B to go back to the main menu (which is the UI node).

If the user presses p/P, then the node enters in the function `setGoal()` which prompts the user to insert valid coordinates, the parameters of the goal are set and the flag `flag`, indicating that a goal has been set, is set to `true`.
A message of type `move_base_msgs/MoveBaseActionGoal` is then published to the topic `/move_base/goal`.

Also, in this function the node subscribes to two topics:
1. `/move_base/status`, which gives us the status of the current goal;
2. `/move_base/feedback`, which gives us the ID of the current goal.

When the robot reaches the goal, than the callback function relative to the `/move_base/status` topic calls the function `cancelGoal()`, which cancels the current goal by publishing a message of type `actionlib_msgs/GoalID` and to the topic `/move_base/cancel`, sets `flag` to false and shuts down the subscriber to the `/move_base/status` topic.

If the user presses c/C, then the node calls the function `cancelGoal()`, which will work only if `flag` is set to `true`, otherwise the node will inform the user that no goal has been set yet.

## Keyboard node
This node lets the user drive manually the robot and it is launched with the file `keyboard.launch`. This was done because I decided to use the `teleop_twist_keyboard` node to drive the robot, so both this node and the keyboard node must be launched together.

At first the user interface informs the user that they can start driving the robot with the teleop_twist_interface, because the default modality is the non-assisted modality.

Then a menu is printed:
- press a/A to switch to the assisted modality, so that the robot won't autonomously hit the walls;
- press n/N to go back to the default modality;
- press b/B to go back to the main menu (which is the UI node).

If the user presses a/A, then the flag `assisted` indicating that the assisted modality has been chosen becomes `true`.

The node subscribes to the topic `/scan`, which checks the laser scanners of the robot, and I implemented the callback function in such away that if `assisted` is set to `false` then it does nothing, while if it is set to `true` it keeps checking whether the robot is within a certain threshold from the walls:
- if there is a wall in front of the robot, the linear velocity is set to 0;
- if there is a wall on one side of the robot, it turns autonomously the other way to avoid hitting the wall.

After this the velocity is restored to its previous value.

If the user presses n/N, the flag `assisted` is set to `false`.

## Flowcharts
The following flowcharts show the behaviour of the two main nodes: the reachPoint node and the keyboard node.
keyboard node             |  reachPoint node
:-------------------------:|:-------------------------:
![keyboard](https://user-images.githubusercontent.com/62473854/160603536-fd62a05f-99c0-45af-b093-02db0d73329a.png)  |  ![reachPoint](https://user-images.githubusercontent.com/62473854/160603551-cbafeff7-83e5-40ed-8b4f-c86a2a0f556d.png)

## Improvements
How the nodes print on the terminal information to the user is not optimized, so sometimes it can be confusing. This could be a possible improvement.

Regarding the reachPoint node, when giving as input a point, the check to see whether it is reachable or not is done after publishing the goal, so an improvement could be doing this check before publishing the goal.

Also, a check to see whether there is already a goal set could be added, so that if the robot is still trying to reach a point, the user won't be able to set a new goal.

