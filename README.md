# Assignment1_rt: Turtle Simulation Project

## Overview

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
   - Stops a turtle if it gets too close to the other turtle (based on a threshold of 2.0 meters).
   - Stops a turtle if it moves too close to the boundaries of the environment (i.e., the position exceeds a limit of `x` or `y > 10.0` or `x` or `y < 1.0).

## Prerequisites
- ROS installed (Noetic).
- turtlesim package installed.
