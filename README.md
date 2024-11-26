# Assignment1_rt: Turtle Simulation Project

## Overview:

This repository named `assignment1_rt` implements two ROS nodes for controlling two turtles velocities, the distance between them and from borders in a simulated environment using the `turtlesim` package.

### Features:
1. **UI Node**:
   - Allows the user to control two turtles (`turtle1` and `turtle2`) via a simple command-line interface.
   - The user can input linear and angular velocities for the selected turtle.
   - The command is applied for 1 second, after which the turtle stops moving.
   - The user can input new commands to control the turtles again.

2. **Distance Node**:
   - Monitors the relative distance between `turtle1` and `turtle2`.
   - Publishes the distance between the turtles to a topic (`/turtle_distance`).
   - Repulse turtles if they get too close to the each other (based on a threshold of 2.0 meters).
   - Return the moving turtle backward if it moves too close to the boundaries of the environment (i.e., the position exceeds a limit of `x` or `y > 10.0` or `x` or `y < 1.0).

## Prerequisites:
- ROS installed (Noetic).
- turtlesim package installed.

## Setup : 
- we Used the previous workspace `my_ros`. 
- we created a new package using `catkin_create_pkg assignment std_msgs roscpp rospy`.
- we created two nodes in the `src` directory named `UI` and `Distance` using cpp and modify the `CMakeLists.txt` file accordingly, doing the same using python in the `scripts` directory.

## File structure:

├─assignment1_rt/ <br>
├── CMakeLists.txt <br>
├── package.xml <br>
├── src/ <br>
├    ├── ui.cpp <br>
├    └── distance.cpp <br>
└── scripts/ <br>
     ├── ui.py <br>
     └── distance.py <br>
     
### Key Components:  
      
## 1. **UI Node**
- **ROS Publisher**:
  - `ros::Publisher pub`: Publishes `geometry_msgs::Twist` messages to control the velocity of a selected turtle (`turtle1` or `turtle2`).

- **getUserInput Function**:
  - Collects user input for selecting which turtle to control (`turtle1` or `turtle2`), and the desired linear and angular velocities.
  
  
## 2. **Distance Node**
- **ROS Publisher**:
  - `ros::Publisher cmd_vel_pub_1`, `cmd_vel_pub_2`: Publish stop commands to the turtles (`turtle1` and `turtle2`) when needed.
  - `ros::Publisher distance_pub`: Publishes the distance between the two turtles.

- **ROS Subscriber**:
  - Subscribes to the `/turtle1/pose` and `/turtle2/pose` topics to receive the position (`turtlesim::Pose`) of each turtle.


## Example of running both nodes:
1. **Lunch** `turtlesim`**:** <br>
Before running the two nodes we need to start roscore: 
```bash
roscore
```
We open another terminal and run the `turtlesim` node : 
```bash
rosrun turtlesim turtlesim_node
```
2. **Run the UI Node:**
- In cpp : <br>
```bash
rosrun assignment1_rt UI_node
```
- In python : <br>
```bash
rosrun assignment1_rt UI.py
```
3. **Run the Distance Node:**
- In cpp : <br>
```bash
rosrun assignment1_rt Distance_node
```
- In python : <br>
```bash
rosrun assignment1_rt Distance.py
```
- Illustration 1: Turtles repulse each other or boundries 
![Example GIF](images/repulsing.gif)

- Illustration 2: Stop when they are close to each other or boundries (Previous commit of the distance code)
![Example GIF](images/just_stopping.gif)
