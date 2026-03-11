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

- A **TurtleBot mobile base**
- A **robotic manipulator arm**
- A **Gazebo-based simulation environment**

## Scope

This project focuses on the integration of **perception, estimation, planning, and actuation** modules required for mobile manipulation.

The robotic platform consists of:

- A **TurtleBot mobile base**
- A **Robotic manipulator arm**
- A **Gazebo-based simulation environment**

### Perception System

The perception stack combines:

- **Beacon-based localization** to estimate the global position of the ball  
- **Camera-based vision module** to refine the ball’s local pose for manipulation  

### System Capabilities

The system implements:

- Autonomous Navigation to approach the ball  
- Precise base alignment to ensure the object lies within the manipulator workspace  
- A custom dynamic throwing mechanism to return the ball to a target location  

### Custom Algorithmic Modules

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

The platform consists of a **TurtleBot mobile base** integrated with a **robotic manipulator arm**, enabling autonomous object retrieval and dynamic throwing.

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

---
## System Architecture
<img width="1690" height="4159" alt="mermaid-diagram" src="https://github.com/user-attachments/assets/6f018392-76c6-4b76-8c53-df585cb9737e" />



## System Modules
<img width="1125" height="440" alt="Sysm Arch table" src="https://github.com/user-attachments/assets/d6f1bf2b-06b7-41f0-8607-a194c7865046" />


### Custom Modules

#### CG Stability Controller

Dynamically adjusts the **manipulator posture** to maintain stable center-of-gravity positioning during base motion.

**Problem:** When the TurtleBot accelerates or decelerates, inertial forces shift the combined center of mass of the base-arm system, reducing traction and destabilizing the robot.

**Solution:** The controller estimates the projected center of gravity and adjusts the arm configuration to keep the CoM within a stable support region — reducing load transfer and improving traction consistency at the drive wheels.

**Center of Gravity Estimation:**

$$x_{cg} = \frac{\sum m_i x_i}{\sum m_i}$$

---

####  Vacuum Throw Release Controller

Implements a custom **dynamic manipulation algorithm** for throwing the ball using a vacuum suction gripper.

**How it works:**
1. The manipulator performs a forward slinging motion, accelerating the end effector
2. The algorithm monitors the arm trajectory
3. At the moment of **maximum tangential velocity** aligned with the desired throw direction, vacuum suction is disengaged
4. The ball detaches and travels along a **ballistic trajectory**

**Key equations:**

Tangential velocity:
$$v_t = r\omega$$

Projectile range approximation:
$$R = \frac{v^2 \sin(2\theta)}{g}$$
By synchronizing release timing with arm motion, the system maximizes throwing **range and repeatability**.

---

<div align="center">

</div>
