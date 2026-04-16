---
layout: default
title: Milestone 2
parent: Project
nav_order: 2
---

# Milestone 2 — Mid-Point Technical Proof
{: .no_toc }

**Due:** Apr 15 &nbsp;|&nbsp; **Weight:** 10% &nbsp;|&nbsp; **Team:** Varad Jahagirdar, Dhiren Makwana, Sharat Mylavarapu

---

<details open markdown="block">
  <summary>Table of Contents</summary>
  {: .text-delta }
- TOC
{:toc}
</details>

---

## 1. Kinematics

### 1.1 Differential Drive Motion Model

The LoCoBot uses a **Kobuki differential drive base**. The robot's state is defined as:

$$\mathbf{x} = [x, y, \theta]^T$$

where $x$, $y$ are the 2D position in the world frame and $\theta$ is the heading angle.

Given left and right wheel velocities $v_L$ and $v_R$, the linear and angular velocities of the robot are:

$$v = \frac{v_R + v_L}{2}, \qquad \omega = \frac{v_R - v_L}{L}$$

where $L = 0.23$ m is the wheel separation (track width of the Kobuki base).

The discrete-time state update equations are:

$$x_{t+1} = x_t + v \cos(\theta_t) \cdot \Delta t$$

$$y_{t+1} = y_t + v \sin(\theta_t) \cdot \Delta t$$

$$\theta_{t+1} = \theta_t + \omega \cdot \Delta t$$

This is the standard **unicycle model** for differential drive robots. In our ROS 2 implementation, this model is handled by the Gazebo Harmonic **DiffDrive** plugin which integrates these equations at every simulation timestep and publishes odometry on `/odom`.

### 1.2 Odometry State Estimation

The odometry is published as a `nav_msgs/msg/Odometry` message with:

$$\hat{\mathbf{x}}_t = \hat{\mathbf{x}}_{t-1} + \begin{bmatrix} v\cos\hat{\theta} \\ v\sin\hat{\theta} \\ \omega \end{bmatrix} \Delta t$$

The `odom_tf_broadcaster` node subscribes to `/odom` and re-publishes the transform `odom → locobot/base_footprint` to the TF tree using sim time, ensuring timestamp consistency with Nav2.

### 1.3 Beacon Trilateration Model

Given 3 beacons at known positions $(x_i, y_i)$ with noisy range measurements $\tilde{d}_i$, the ball position $(x_b, y_b)$ is estimated by solving:

$$\mathbf{A} \mathbf{p} = \mathbf{b}$$

where:

$$A = 2\begin{bmatrix} x_2 - x_1 & y_2 - y_1 \\ x_3 - x_1 & y_3 - y_1 \end{bmatrix}, \quad b = \begin{bmatrix} \tilde{d}_1^2 - \tilde{d}_2^2 - x_1^2 + x_2^2 - y_1^2 + y_2^2 \\ \tilde{d}_1^2 - \tilde{d}_3^2 - x_1^2 + x_3^2 - y_1^2 + y_3^2 \end{bmatrix}$$

The noisy range measurements are modeled as:

$$\tilde{d}_i = d_i + \mathcal{N}(0, \sigma^2), \quad \sigma = 0.3 \text{ m}$$

---

## 2. System Architecture

### 2.1 rqt_graph

<img width="1755" height="1240" alt="rosgraph_page-0001" src="https://github.com/user-attachments/assets/f4f1548a-8e34-46c0-8718-29801368f96c" />

*Figure 1: ROS 2 node graph showing all active nodes and topic connections during full system operation.*

### 2.2 System Mermaid Diagram

<img width="1247" height="779" alt="Screenshot 2026-04-15 203706" src="https://github.com/user-attachments/assets/fa850897-a2cf-4794-b475-3a3385e3e66c" />


### 2.3 Active Topics

| Topic | Message Type | Publisher | Subscriber |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | gz_bridge | odom_tf_broadcaster, Nav2 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 | gz_bridge → Gazebo |
| `/scan` | `sensor_msgs/msg/LaserScan` | depthimage_to_laserscan | Nav2 costmap |
| `/camera/depth` | `sensor_msgs/msg/Image` | depth_bridge | depthimage_to_laserscan |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | depth_bridge | depthimage_to_laserscan |
| `/robot_description` | `std_msgs/msg/String` | robot_state_publisher | Gazebo spawn |
| `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_publisher | robot_state_publisher |
| `/tf` | `tf2_msgs/msg/TFMessage` | gz_bridge, odom_tf_broadcaster, static_transform_publisher | Nav2, RViz2 |
| `/ball_global_pose` | `geometry_msgs/msg/PoseStamped` | beacon_localization | Nav2 goal |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | user/nav node | Nav2 bt_navigator |

---

## 3. Module Descriptions

### 3.1 Module Declaration Table

| Module | Type | Status | Plan |
|---|---|---|---|
| Beacon Localization | Custom | ✅ Complete | Gaussian noise + trilateration implemented |
| Vision-Based Ball Detection | Custom | 🔄 Pending | M3 Final |
| Autonomous Navigation | Library (Nav2) | ✅ Complete | MPPI controller configured |
| Base Alignment | Custom | 🔄 Pending | M3 Final |
| Manipulation & Grasping | Library (MoveIt2) | 🔄 Pending | M3 Final |
| CG Stability Controller | Custom | 🔄 Pending | M3 Final |
| Vacuum Throw Release | Custom | 🔄 Pending | M3 Final |
| Gazebo Simulation | Library | ✅ Complete | World with beacons, obstacles, ball, throw target |
| Depth Camera + LaserScan | Library | ✅ Complete | Added to URDF, bridged to ROS2 |
| TF Tree Management | Library | ✅ Complete | odom→base_footprint via gz_bridge |

### 3.2 Nav2 — Autonomous Navigation Stack

Nav2 provides the full autonomous navigation pipeline. Key parameters tuned:

| Parameter | Value | Purpose |
|---|---|---|
| `inflation_radius` | 0.15 m | Reduced to allow navigation through narrow gaps |
| `robot_radius` | 0.15 m | Robot footprint approximation |
| `vx_max` | 0.8 m/s | Maximum forward velocity |
| `xy_goal_tolerance` | 0.15 m | Goal reached threshold |
| `controller_frequency` | 20 Hz | MPPI controller update rate |
| `scan_topic` | `/scan` | LaserScan input from depth camera |
| `global_frame` | `map` | Planning frame |
| `robot_base_frame` | `locobot/base_footprint` | Robot frame |

**Planner:** NavFn (Dijkstra-based global planner)  
**Controller:** MPPI (Model Predictive Path Integral)  
**Config:** [`nav2_params.yaml`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_gazebo/config/nav2_params.yaml)

### 3.3 depthimage_to_laserscan

Converts depth camera image to 2D LaserScan for Nav2 costmap updates.

| Parameter | Value | Purpose |
|---|---|---|
| `scan_height` | 10 pixels | Rows used for scan |
| `range_min` | 0.2 m | Minimum valid range |
| `range_max` | 10.0 m | Maximum valid range |
| `output_frame` | `locobot/depth_camera_link` | Output frame matching TF tree |
| `use_sim_time` | true | Sync with Gazebo sim time |

### 3.4 odom_tf_broadcaster

**Source:** [`odom_tf_broadcaster.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/odom_tf_broadcaster.py)

Subscribes to `/odom` and broadcasts the `odom → locobot/base_footprint` TF transform using the odometry timestamp to ensure Nav2 receives transforms with correct sim time.

### 3.5 beacon_localization

**Source:** [`beacon_localization.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/beacon_localization.py)

Subscribes to Gazebo pose info, extracts ball position, injects Gaussian noise into beacon range measurements, solves trilateration, and publishes `/ball_global_pose`.

---

## 4. Experimental Analysis & Validation

### 4.1 Depth Camera Calibration

The depth camera is positioned at (0.1, 0, 0.4) m relative to `locobot/base_link`, facing forward with a 60° horizontal FOV. The `depthimage_to_laserscan` node extracts a 10-pixel horizontal strip from the depth image and converts it to a 2D LaserScan. Range is validated between 0.2 m and 10.0 m, publishing at ~8 Hz.

### 4.2 Beacon Noise & Uncertainty Analysis

<!-- VARAD: Add your noise analysis table and results here -->


## 4.3 Run-Time Issues & System Behaviors

The following run-time issues were observed and resolved during development
and testing of the autonomous navigation system:

### Issue 1: TF Tree Disconnection

**Observed Behavior:**
Nav2 repeatedly logged:
```
Could not find a connection between 'odom' and 'locobot/base_footprint'
because they are not part of the same tree.
```

**Root Cause:**
The Gazebo-ROS2 bridge was publishing odometry on `/odom` but not
forwarding the corresponding TF transform to the ROS2 `/tf` topic.

**Resolution:**
Added `/model/locobot/tf` to the gz_bridge with a remapping to `/tf`,
ensuring the `odom → locobot/base_footprint` transform reaches Nav2
with correct simulation timestamps.

---

### Issue 2: Scan Frame Timestamp Mismatch

**Observed Behavior:**
Nav2 costmap continuously dropped scan messages:
```
Message Filter dropping message: frame 'camera_depth_frame'
at time X for reason 'timestamp earlier than transform cache'
```

**Root Cause:**
The `depthimage_to_laserscan` node published the LaserScan with a
default frame `camera_depth_frame` which did not exist in the TF tree.

**Resolution:**
Set the `output_frame` parameter to `locobot/depth_camera_link` which
is the actual TF frame of the depth camera in the robot URDF.

---

### Issue 3: Robot Spawning at Previous Position

**Observed Behavior:**
After restarting the simulation, the robot appeared at its last position
from the previous run, causing odometry to start from a non-zero position
and breaking Nav2's localization.

**Root Cause:**
Gazebo Harmonic caches the last known model state between runs.

**Resolution:**
Added the `-r` flag to `gz_sim` to auto-reset the world on launch,
and added a 5-second `TimerAction` delay before robot spawn to ensure
Gazebo fully initializes first.

---

### Issue 4: Robot Stuck Near Obstacles

**Observed Behavior:**
The robot stopped moving near clusters of obstacles even when a
navigable path existed nearby. It would remain stuck for extended
periods before the recovery behavior triggered.

**Root Cause:**
The Nav2 costmap inflation radius was set to 0.55 m, causing obstacles
to appear artificially large and blocking all available paths through
the cluttered environment.

**Resolution:**
Reduced inflation parameters in `nav2_params.yaml`:
- `inflation_radius`: 0.55 m → 0.15 m
- `robot_radius`: 0.22 m → 0.15 m

This allowed the planner to find paths through the gaps between obstacles.

---
---

## 5. Milestone Video

**Navigation Demo:** https://youtu.be/GvLrjGdQ7WQ

The video demonstrates the robot autonomously navigating from its spawn position to the ball at (3.0, 3.0) in Gazebo, with the depth camera LaserScan visible in RViz2 and obstacle avoidance active throughout.

---

## 6. Instructor Feedback Integration (M1 → M2)

| M1 Feedback | Technical Action Taken |
|---|---|
| Missing module intents and safety | Added full Module Declaration Table with Intent, Library, Safety for all modules |
| Index page placeholder | Updated index.md with team info and project description |
| Flowchart `\n` characters | Fixed Mermaid diagram syntax |
| Git repo not found | Created `ros2_ws/` directory with all ROS2 packages |
| Beacon noise? | Implemented Gaussian noise $\tilde{d}_i = d_i + \mathcal{N}(0, \sigma^2)$ in beacon_localization.py |
| Why Gazebo? | Evaluated Gazebo vs Webots vs Isaac Sim — selected for ROS2 Jazzy compatibility |
| Which robot platform? | Committed to LoCoBot (Kobuki base + WidowX 250s arm) |
| Base velocity during throw | Incorporated into throw design: $v_{release} = v_{arm} + v_{base}$ |

---

## 7. Individual Contribution

| Team Member | Role | Key Commits | Files |
|---|---|---|---|
| Dhiren Makwana | Simulation & Navigation | [`435758f`](https://github.com/Varad1722/Mobile_Robotics/commit/435758f) [`f054235`](https://github.com/Varad1722/Mobile_Robotics/commit/f054235) [`a837ffa`](https://github.com/Varad1722/Mobile_Robotics/commit/a837ffa) | [`locobot_gazebo.launch.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_gazebo/launch/locobot_gazebo.launch.py) [`nav2_params.yaml`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_gazebo/config/nav2_params.yaml) [`locobot_kobuki.urdf.xacro`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/interbotix_xslocobot_descriptions/urdf/kobuki_version/locobot_kobuki.urdf.xacro) |
| Varad Jahagirdar | Perception & Localization | [`cfed6b9`](https://github.com/Varad1722/Mobile_Robotics/commit/cfed6b9) [`bab74f0`](https://github.com/Varad1722/Mobile_Robotics/commit/bab74f0) | [`beacon_localization.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/beacon_localization.py) [`locobot_world.sdf`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/interbotix_xslocobot_descriptions/worlds/locobot_world.sdf) |
| Sharat Mylavarapu | Documentation & Integration | [`20c1733`](https://github.com/Varad1722/Mobile_Robotics/commit/20c1733) | [`report2.md`](https://github.com/Varad1722/Mobile_Robotics/blob/main/project/report2.md) |
