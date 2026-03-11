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
