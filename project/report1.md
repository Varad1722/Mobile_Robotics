---
layout: default
title: Milestone 1
parent: Project
nav_order: 1
---

# Milestone 1

# Mission Statement & Scope

## Mission Statement

The mission of this project is to develop an autonomous **mobile manipulation system** capable of locating, retrieving, and dynamically throwing a ball in a simulated environment.  

The system integrates:

- Global beacon-based localization  
- Vision-based object detection  
- Autonomous navigation  
- Robotic manipulation  

All components operate within a **ROS 2 and Gazebo simulation framework**.

The robot must autonomously:

1. Detect the ball  
2. Navigate toward it  
3. Pick it up using a vacuum-based gripper  
4. Execute a dynamic throwing action toward a predefined target location  

---

## Scope

This project focuses on the integration of **perception, estimation, planning, and actuation** modules required for mobile manipulation.

The robotic platform consists of:

- A **TurtleBot mobile base**
- A **robotic manipulator arm**
- A **Gazebo-based simulation environment**

### Perception System

The perception stack combines:

- **Beacon-based localization** to estimate the global position of the ball  
- **Camera-based vision module** to refine the ball’s local pose for manipulation  

### System Capabilities

The system implements:

- Autonomous navigation to approach the ball  
- Precise base alignment to ensure the object lies within the manipulator workspace  
- A custom dynamic throwing mechanism to return the ball to a target location  

**Equations:**

Two task-specific control algorithms are introduced:

- **Center-of-Gravity (CG) Stability Controller**  
  Dynamically adjusts manipulator posture during base motion to maintain system stability.

- **Vacuum Throw Release Controller**  
  Synchronizes suction release with peak tangential velocity during the arm swing to maximize throwing performance.

### Environment Assumptions

The current system operates in an **open indoor simulated environment without obstacles**.  

This allows the project to focus on subsystem integration and coordination. Future extensions may introduce:

- Obstacle-rich environments  
- More advanced manipulation tasks  

---

# Technical Specifications

## Robot Platform

The robotic system is implemented using:

- **ROS 2** as the middleware framework  
- **Gazebo** for physics-based simulation  

The platform consists of a **TurtleBot mobile base** integrated with a **robotic manipulator arm**, enabling autonomous object retrieval and dynamic throwing.

## Software Frameworks

The system integrates the following ROS 2 packages and tools:

- **Gazebo** – Physics-based simulation environment  
- **ROS 2** – Distributed robotic communication framework  
- **Nav2** – Autonomous navigation stack  
- **MoveIt2** – Manipulation planning and trajectory execution  
- **TF2** – Coordinate frame transformation system  
- **RViz2** – Visualization and debugging interface  
- **OpenCV** – Camera-based object detection  

The robot operates in an open environment to simplify navigation and emphasize perception–manipulation integration.


# High-Level System Architecture

The system follows a **Perception → Estimation → Planning → Actuation** pipeline.

1. **Beacon localization** provides a coarse estimate of the ball position.  
2. A **goal generation module** computes the navigation target.  
3. The **Nav2 navigation stack** drives the robot toward the ball.  
4. The **vision module** detects and refines the ball pose.  
5. The **base alignment module** positions the robot for grasping.  
6. The **manipulator** grasps the ball using a vacuum gripper.  
7. The **throwing planner** computes a slinging trajectory.  
8. The **release controller** disengages suction at the optimal moment.

## System Architecture

```mermaid
graph TD

A[Gazebo Simulation World<br/>Indoor Environment<br/>TurtleBot + Manipulator<br/>Ball with Beacon<br/>Camera + LiDAR Sensors<br/>Pickup Zone<br/>Throw Zone / World Home]

A --> B[ros_gz_bridge<br/>ROS2 - Gazebo Integration]

B --> C[Robot Model / TF Layer<br/>robot_state_publisher<br/>joint_states<br/>TF frames]

C --> D[Beacon Localization Node<br/>Estimate global ball pose<br/>Publishes /ball_global_pose]

C --> F[Ball Vision Node<br/>Detect ball using camera<br/>Publishes /ball_local_pose]

D --> E[Ball Goal Generator Node<br/>Compute staging pose<br/>Send navigation goal]

E --> G[Nav2 Navigation Stack<br/>map_server<br/>amcl<br/>planner_server<br/>controller_server<br/>bt_navigator]

G --> H[Base Alignment Node<br/>Fine-align robot with ball<br/>Ensure ball reachable]

F --> H
D --> H

H --> I[Grasp Pose Generator<br/>Generate pre-grasp<br/>grasp and lift poses]

I --> J[MoveIt2 Manipulation Layer<br/>move_group<br/>controller_manager<br/>joint_trajectory_controller<br/>gripper_controller]

J --> K[Pickup Execution<br/>Pick up and hold ball]

K --> L[Task Planner Node<br/>State Machine Controller]

L --> M[Navigate to Throw Position]

M --> N[Throw Planner Node<br/>Generate throw trajectory<br/>Compute release timing]




N --> O[Ball Release Controller<br/>Reduce grip / open gripper<br/>at release point]

O --> P[Ball Thrown Toward<br/>World Home Position]

## System Modules

| Module Name | Type | Description |
|--------------|------|-------------|
| Gazebo Simulation World | Library | Gazebo world with TurtleBot, arm, ball, sensors, pickup zone, and throw target. |
| ros_gz_bridge | Library | Bridge enabling communication between Gazebo simulation and ROS2 nodes. |
| Robot Model / TF Layer | Library | Publishes robot joint states and TF frames for kinematic transformations. |
| Beacon Localization Node | Library | Estimates global ball position in the map frame using beacon signals. |
| Ball Vision Node | Library | Detects the ball using camera input and publishes its pose relative to the robot. |
| Ball Goal Generator Node | Library | Computes a navigation staging goal near the ball. |
| Nav2 Navigation Stack | Library | Provides localization, path planning, and motion control for the mobile base. |
| Base Alignment Node | Library | Performs fine base positioning so the ball lies within arm reach. |
| Grasp Pose Generator Node | Library | Generates pre-grasp, grasp, and lift poses for the manipulator. |
| MoveIt2 Manipulation Layer | Library | Handles arm motion planning and trajectory execution. |
| Pickup Execution Node | Library | Executes arm motion and activates the vacuum gripper to pick the ball. |
| Task Planner Node | Library | Coordinates navigation, pickup, and throwing tasks. |
| Navigate to Throw Position | Library | Moves the robot to the predefined throwing location. |
| CG Stability Controller | Custom | Adjusts arm posture during motion to maintain stable center of gravity. |
| Throw Planner Node | Custom | Computes the throwing trajectory and optimal release timing. |
| Ball Release Controller Node | Custom | Releases the ball from the vacuum gripper at the correct moment. |
