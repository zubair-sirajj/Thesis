# Autonomous Navigation Planner Comparison Using ROS 2, Nav2, and Gazebo

This repository contains the implementation files used for the Bachelor’s thesis:

**Implementation and Evaluation of Autonomous Navigation for a Mobile Robot Using ROS 2 and Gazebo**

The thesis compares the performance of three path planning approaches for autonomous mobile robot navigation:

- Dijkstra
- A*
- Hybrid A*

The experiments were conducted using **ROS 2**, **Nav2**, **Gazebo**, and a simulated **TurtleBot3** robot. The planners were tested in multiple custom static simulation environments, including base, narrow-corridor, dead-end, sparse-obstacle, dense-obstacle, and combined-challenge scenarios.

## Project Overview

The purpose of this project was to evaluate how different global planners perform under controlled simulation conditions. The comparison focused on:

- navigation success or failure
- navigation time
- path length
- replanning events
- planner behaviour in different static environments

The study does not develop a new path planning algorithm. Instead, it configures and evaluates existing planners within the Nav2 framework.

## Software and Tools

The implementation used the following main tools:

- ROS 2 Kilted Kaiju
- Nav2 1.4.2
- Gazebo 9.5.0
- TurtleBot3 simulation
- slam_toolbox
- Python
- Bash
