---
layout: default
title: Milestone 3
parent: Project
nav_order: 3
---

# Milestone 3 — Final Documentation & Analysis
{: .no_toc }

**Due:** May 8 &nbsp;|&nbsp; **Weight:** 15% &nbsp;|&nbsp; **Team:** Varad Jahagirdar, Dhiren Makwana, Sharat Mylavarapu

---

<details open markdown="block">
  <summary>Table of Contents</summary>
  {: .text-delta }
- TOC
{:toc}
</details>

---

## 1. Graphical Abstract

<!-- PLACEHOLDER: Insert graphical abstract image here -->
<!-- Recommended: One figure showing mission overview — robot navigating to ball, grasping, returning to target -->
<!-- Example: ![Graphical Abstract](../assets/graphical_abstract.png) -->

> **Mission:** An autonomous mobile manipulation system built on the LoCoBot (Kobuki + WidowX 250s) navigates a cluttered arena, detects a magenta ball using a color camera and depth sensor, grasps it using geometric inverse kinematics, and returns to a throw target at the arena center — all within a single ROS 2 pipeline running in Gazebo Harmonic.

---

## 2. Algorithm

### 2.1 Full State Machine

The system runs a single shared `/robot_state` topic as a finite state machine. Every node listens and reacts to state transitions.

<img width="330" height="750" alt="Screenshot 2026-05-08 120334" src="https://github.com/user-attachments/assets/45dd2805-d623-428c-a775-93f8bf829863" />


### 2.2 Beacon Trilateration — Varad Jahagirdar

Beacon Localization

Already fully documented in Milestone 2. The node continues to operate identically in M3.

**Source:** [`beacon_localization.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/beacon_localization.py)

Performance across 10 M3 trials: mean error **0.318 m**, consistent with $\sigma = 0.3$ m injected noise.

---


### 2.3 Base Alignment Controller — Dhiren Makwana

The base alignment node uses Gazebo ground truth poses and a proportional controller to rotate and drive toward the ball:

**Angular error:**

$$e_\theta = \text{atan2}(b_y - r_y,\ b_x - r_x) - \theta_{\text{robot}}$$

normalized to $[-\pi, \pi]$.

**Control law:**

$$\omega = 0.5 \cdot e_\theta \quad \text{(rotate phase)}$$

$$v = 0.1\ \text{m/s}, \quad \omega = 0.3 \cdot e_\theta \quad \text{(drive phase)}$$

Alignment is declared complete when $d < 0.535$ m and $|e_\theta| < 0.1$ rad.

**Source:** [`base_alignment.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/base_alignment.py#L84)

### 2.4 Ball Detection — Dhiren Makwana

The color camera detects the magenta ball using a dual-range HSV mask:

$$\text{mask} = \text{inRange}(\text{HSV}, [140,100,100], [180,255,255])\ \cup\ \text{inRange}(\text{HSV}, [0,100,100], [10,255,255])$$

The largest contour centroid gives pixel coordinates $(u, v)$. The depth value $z$ at that pixel is read from the depth image. The 3D ball position in camera frame is computed using the pinhole camera model:

$$x_{\text{cam}} = \frac{(u - c_x) \cdot z}{f_x}, \qquad y_{\text{cam}} = \frac{(v - c_y) \cdot z}{f_y}$$

where $f_x = f_y = 277.19$ px, $c_x = 160$, $c_y = 120$ (320×240 image).

Published to `/ball_camera_pos` as a `PointStamped` in the camera frame.

**Source:** [`ball_detection.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/ball_detection.py#L68)

### 2.5 Arm Grasp & Geometric Inverse Kinematics — Varad Jahagirdar

The `arm_grasp` node activates when `/robot_state` receives `ALIGNED`. It tilts the camera down to observe the ball at close range, computes arm joint angles using geometric inverse kinematics, and executes a pre-grasp → grasp → lift sequence.

**Source:** [`arm_grasp.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/arm_grasp.py)

#### Grasp Sequence

```
ALIGNED received
    → tilt camera down (0.75 rad)
    → wait 2s for camera to stabilize
    → get ball 3D position (camera depth or Gazebo GT fallback)
    → compute geometric IK
    → open gripper (0.037 m)
    → rotate waist to face ball
    → move to pre-grasp (5cm above ball)
    → lower to grasp position
    → close gripper (0.015 m)
    → lift to pre-grasp height
    → return to arm home pose
    → publish GRASPED
```

#### Ball Position in Arm Frame

**From camera (primary):**

The depth camera is tilted down by $\alpha = 0.75$ rad. The ball position in camera frame $(x_c, y_c, z_c)$ — obtained from the depth unprojection in `ball_detection.py` — is rotated into arm base frame via a Y-axis rotation:

$$x_{\text{arm}} = x_{\text{cam,offset}} + z_c \cos\alpha - y_c \sin\alpha$$

$$y_{\text{arm}} = x_c$$

$$z_{\text{arm}} = z_{\text{cam,offset}} - z_c \sin\alpha - y_c \cos\alpha$$

Camera offset from arm base link (verified via TF at tilt=0): $x_{\text{cam,offset}} = 0.003$ m, $z_{\text{cam,offset}} = 0.302$ m.

**From Gazebo GT (fallback):**

If camera depth is unavailable, ball position is computed from live Gazebo ground truth poses. The world-frame vector from robot to ball is rotated into the robot local frame using the live yaw $\psi$, then converted to shoulder frame:

$$x_{\text{arm}} = (b_x - r_x)\cos(-\psi) - (b_y - r_y)\sin(-\psi) - d_{\text{arm}}$$

$$y_{\text{arm}} = (b_x - r_x)\sin(-\psi) + (b_y - r_y)\cos(-\psi)$$

$$z_{\text{arm}} = b_z - (h_{\text{base}} + h_{\text{shoulder}})$$

where $\psi$ is robot yaw, $d_{\text{arm}} = 0.140$ m (arm base forward offset from robot center), $h_{\text{base}} = 0.108$ m, $h_{\text{shoulder}} = 0.354825$ m.

#### Geometric Inverse Kinematics

The WidowX 250s arm is treated as a 2-link planar manipulator in the sagittal plane (x-z). All link lengths are derived directly from the URDF:

$$L_1 = \sqrt{0.04975^2 + 0.25^2} = 0.2549\ \text{m} \quad \text{(shoulder to elbow)}$$

$$L_2 = 0.175 + 0.075 = 0.250\ \text{m} \quad \text{(elbow to wrist)}$$

$$L_3 = 0.065 + 0.043 + 0.023 + 0.027575 = 0.1586\ \text{m} \quad \text{(wrist to gripper tip)}$$

**Wrist target** — subtract gripper length from horizontal reach so IK targets the wrist, not the tip:

$$w_x = x_{\text{arm}} - L_3, \qquad w_z = -z_{\text{arm}}$$

**Reach distance:**

$$d = \sqrt{w_x^2 + w_z^2}, \qquad d \leq L_1 + L_2$$

**Elbow angle** via law of cosines:

$$\theta_{\text{elbow}} = \cos^{-1}\!\left(\frac{d^2 - L_1^2 - L_2^2}{2 L_1 L_2}\right)$$

**Shoulder angle** — angle to wrist target minus correction for elbow bend:

$$\alpha = \text{atan2}(w_z,\ w_x)$$

$$\beta = \cos^{-1}\!\left(\frac{d^2 + L_1^2 - L_2^2}{2\, d\, L_1}\right)$$

$$\theta_{\text{shoulder}} = \alpha - \beta$$

**Wrist compensation** — keeps gripper roughly horizontal throughout motion:

$$\theta_{\text{wrist}} = -(\theta_{\text{shoulder}} + \theta_{\text{elbow}})$$

**Waist angle** — horizontal rotation to align arm plane with ball:

$$\theta_{\text{waist}} = \text{atan2}(y_{\text{arm}},\ x_{\text{arm}}), \qquad \theta_{\text{waist}} \in \left[-\frac{\pi}{2},\ \frac{\pi}{2}\right]$$

#### Pre-Grasp Strategy

To avoid pushing the ball on approach, the arm first moves to a pre-grasp position 5 cm above the ball, then lowers straight down:

$$z_{\text{pre}} = z_{\text{arm}} + 0.05\ \text{m}$$

IK is solved independently for both the pre-grasp and grasp positions. The arm moves elbow → wrist → shoulder in that order to keep the center of mass balanced and prevent the robot from tipping.

#### Joint Control

All joints are commanded directly via Gazebo transport using subprocess calls — bypassing the ROS layer entirely for lower latency:

```python
topic = f"/model/locobot/joint/{joint}/0/cmd_pos"
cmd = ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {angle}"]
subprocess.run(cmd, capture_output=True)
```

Joints controlled: `waist`, `shoulder`, `elbow`, `forearm_roll`, `wrist_angle`, `wrist_rotate`, `left_finger`, `tilt`.

### 2.6 Throw Controller — Sharat Mylavarapu

<!-- SHARAT: Paste your throw controller algorithm and physics equations here -->

---

## 3. System Architecture

### 3.1 ROS 2 Node Graph

<!-- PLACEHOLDER: rqt_graph screenshot -->
<!-- Run: ros2 run rqt_graph rqt_graph -->
<!-- ![ROS Graph](../assets/rqt_graph_m3.png) -->

### 3.2 Full System Diagram

<img width="1331" height="560" alt="Screenshot 2026-05-08 120400" src="https://github.com/user-attachments/assets/1040000f-3b3f-4564-b965-b8faf059b0ba" />


### 3.3 Active Topics

| Topic | Message Type | Publisher | Subscriber |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | gz_bridge | odom_tf_broadcaster, Nav2 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2, base_alignment, arm_grasp | gz_bridge |
| `/scan` | `sensor_msgs/msg/LaserScan` | depthimage_to_laserscan | Nav2 costmap |
| `/camera/depth` | `sensor_msgs/msg/Image` | depth_bridge | ball_detection, arm_grasp |
| `/camera/color` | `sensor_msgs/msg/Image` | depth_bridge | ball_detection |
| `/ball_global_pose` | `geometry_msgs/msg/PoseStamped` | beacon_localization | auto_navigator |
| `/ball_detected` | `std_msgs/msg/Bool` | ball_detection | auto_navigator |
| `/ball_camera_pos` | `geometry_msgs/msg/PointStamped` | ball_detection | arm_grasp |
| `/ball_pixel_pos` | `geometry_msgs/msg/Point` | ball_detection | — |
| `/robot_state` | `std_msgs/msg/String` | auto_navigator, base_alignment, arm_grasp | all nodes |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | auto_navigator | Nav2 bt_navigator |

---

## 4. Benchmarking & Results

### 4.1 Navigation Success Rate (10 Trials)

The full pipeline was tested across 10 independent trials with the ball spawning at random safe positions. Each trial starts from robot spawn at $(0, -4.5)$.

| Trial | Ball Spawn | Navigation | Alignment | Grasp Attempt | Ball Lifted |
|-------|-----------|------------|-----------|---------------|-------------|
| 1 | (3.5, 3.5) | ✅ | ✅ | ✅ | ❌ |
| 2 | (3.5, -3.5) | ✅ | ✅ | ✅ | ❌ |
| 3 | (-3.5, 3.5) | ✅ | ✅ | ✅ | ❌ |
| 4 | (-3.5, -3.5) | ✅ | ✅ | ✅ | ❌ |
| 5 | (1.5, 3.5) | ✅ | ✅ | ✅ | ❌ |
| 6 | (-1.5, 3.5) | ✅ | ✅ | ✅ | ❌ |
| 7 | (1.5, -3.5) | ✅ | ✅ | ✅ | ❌ |
| 8 | (-1.5, -3.5) | ✅ | ✅ | ✅ | ❌ |
| 9 | (3.5, 3.5) | ✅ | ✅ | ✅ | ❌ |
| 10 | (1.5, 3.5) | ✅ | ✅ | ✅ | ❌ |

| Stage | Success Rate |
|-------|-------------|
| Navigation to ball | 10/10 (100%) |
| Base alignment | 10/10 (100%) |
| Grasp attempt (gripper reaches ball) | 10/10 (100%) |
| Ball successfully lifted | 0/10 (0%) |

### 4.2 Beacon Localization Error (10 Trials) — Varad Jahagirdar

| Trial | True Position | Estimated Position | Error (m) |
|-------|--------------|-------------------|-----------|
| 1 | (3.5, 3.5) | (3.71, 3.28) | 0.30 |
| 2 | (3.5, -3.5) | (3.24, -3.71) | 0.34 |
| 3 | (-3.5, 3.5) | (-3.68, 3.29) | 0.28 |
| 4 | (-3.5, -3.5) | (-3.79, -3.34) | 0.33 |
| 5 | (1.5, 3.5) | (1.72, 3.28) | 0.31 |
| 6 | (-1.5, 3.5) | (-1.74, 3.26) | 0.35 |
| 7 | (1.5, -3.5) | (1.29, -3.71) | 0.30 |
| 8 | (-1.5, -3.5) | (-1.71, -3.28) | 0.32 |
| 9 | (3.5, 3.5) | (3.68, 3.73) | 0.29 |
| 10 | (1.5, 3.5) | (1.78, 3.24) | 0.36 |

**Mean error:** 0.318 m — consistent with injected noise $\sigma = 0.3$ m.

### 4.3 Base Alignment Error — Dhiren Makwana

After the robot declares ALIGNED, residual angular and distance errors were measured:

| Trial | Distance at ALIGNED (m) | Angle Error at ALIGNED (deg) |
|-------|------------------------|------------------------------|
| 1 | 0.534 | 0.7 |
| 2 | 0.531 | 1.2 |
| 3 | 0.536 | 0.9 |
| 4 | 0.533 | 1.4 |
| 5 | 0.535 | 0.8 |

**Mean distance:** 0.534 m (target 0.535 m) — alignment is consistent and repeatable.

### 4.4 Grasp Performance Analysis — Varad Jahagirdar

#### Overview

The `arm_grasp` node activates when the `/robot_state` topic receives `ALIGNED`. It tilts the camera down to see the ball at close range, computes arm joint angles using geometric inverse kinematics, and executes a pre-grasp → grasp → lift sequence.

**Source:** [`arm_grasp.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/arm_grasp.py)

#### IK Accuracy

The geometric IK solver computes joint angles from live Gazebo ground truth positions with no hardcoding — the ball and robot positions are read dynamically on every grasp attempt. Across 10 trials with the ball spawning at 8 different arena positions, the gripper successfully reached the ball and positioned the fingers around it in **10/10 trials**.

The IK reach clamping logic ensures graceful degradation: if the computed wrist target exceeds $L_1 + L_2 = 0.5049$ m, the target is scaled to the maximum reachable distance rather than producing invalid joint angles.

#### Gripper Limitation

Force-closure is not achieved due to a known Gazebo Harmonic physics limitation — the mimic constraint for the right finger (`right_finger` mimics `left_finger` with multiplier -1) is not supported by the physics engine, so only the left finger closes. The ball sits correctly between the fingers but is not lifted.

In a real robot deployment using the Interbotix Python API with servo torque control, the gripper would achieve force-closure on the ball through compliant torque-controlled grasping.

#### Grasp Performance Summary

| Metric | Result |
|--------|--------|
| Trials | 10 |
| Gripper reached ball | 10/10 (100%) |
| Ball successfully lifted | 0/10 (0%) |
| Failure cause | Gazebo mimic joint constraint not supported |
| IK solver | Geometric 2-link (law of cosines) |
| Position source | Live Gazebo GT (dynamic, no hardcoding) |
| Pre-grasp offset | 5 cm above ball |

### 4.5 Demo Video

<!-- PLACEHOLDER: Replace with actual YouTube link -->
**Full Mission Demo:** [YouTube Link](https://youtu.be/PLACEHOLDER)

<!-- PLACEHOLDER: Add screenshots -->
<!-- ![Navigation](../assets/nav_screenshot.png) -->
<!-- ![Alignment](../assets/alignment_screenshot.png) -->
<!-- ![Grasp Attempt](../assets/grasp_screenshot.png) -->

---

## 5. Ethical Impact Statement

### 5.1 Privacy

The system uses an onboard RGB-D camera for ball detection. In simulation, this raises no privacy concerns. In a real-world deployment, the camera would capture bystanders in the arena. Mitigation strategies include: limiting the camera field of view to the floor plane only (achieved by tilting the camera downward as implemented), processing images entirely onboard without transmission, and applying person-detection masks to blur any human presence before logging. Future iterations should implement GDPR-compliant data handling by discarding raw frames immediately after detection.

### 5.2 Safety

The WidowX 250s arm has a maximum reach of ~0.5 m and operates at low joint velocities in our implementation (1 joint command per 0.8–1.5 seconds). The kinetic energy at the end effector is minimal. However, the Kobuki base moves at up to 0.8 m/s, which poses a collision risk in human-occupied spaces. Our system uses Nav2 with a 0.15 m inflation radius around obstacles, but this does not account for dynamic obstacles such as people. Future iterations should integrate a LIDAR-based emergency stop and velocity scaling based on proximity to humans. The throw controller must also constrain release velocity to ensure the ball does not exceed safe kinetic energy thresholds in shared spaces.

### 5.3 Bias & Hardware Limitations

The ball detection system relies on a specific magenta HSV color range. This creates a systematic bias — any object with similar color in the environment will be falsely detected. In our arena, all obstacles were deliberately set to grey to avoid false positives, but this would not hold in an uncontrolled environment. The depth camera used for 3D ball localization has a minimum range of 0.2 m, meaning objects closer than 20 cm are invisible. This is a fundamental hardware limitation that affects grasping performance at close range. From a justice perspective, the system is designed for a specific arena configuration — it would not generalize to environments with different lighting conditions, obstacle layouts, or ball colors without retuning the HSV parameters and Nav2 costmap settings.

---

## 6. Module Status

| Module | Status | Notes |
|--------|--------|-------|
| Beacon Localization | ✅ Complete | Gaussian noise + trilateration |
| Nav2 Navigation | ✅ Complete | MPPI controller, obstacle avoidance |
| Ball Detection | ✅ Complete | HSV + depth camera 3D position |
| Base Alignment | ✅ Complete | Gazebo GT, proportional controller |
| Arm Home Pose | ✅ Complete | Safe navigation configuration |
| Random Ball Spawn | ✅ Complete | 8 predefined safe positions |
| Auto Navigator | ✅ Complete | Full state machine integration |
| Arm Grasp | 🔄 Partial | Gripper reaches ball, no force-closure |
| Throw Controller | 🔄 Partial | Sharat's module |

---


## 7. Sharat Mylavarapu Contribution
{: .no_toc }

**Module:** Arm Controller, End Effector Logging, RViz Debugging

---

<details open markdown="block">
  <summary>Table of Contents</summary>
  {: .text-delta }
- TOC
{:toc}
</details>

---

## 7.1. Overview

Sharat Mylavarapu's contribution to Milestone 3 covers the complete arm controller pipeline for the Locobot wx250s in Gazebo simulation. The work includes a Forward Kinematics (FK) based arm throw controller, an end effector logging node for debugging and calibration, and a GUI-to-Gazebo joint state bridge. Together these nodes enable the robot to autonomously pick up a ball and throw it toward a target.

**Commit:** [`8244a94`](https://github.com/Varad1722/Mobile_Robotics/commit/8244a947a0355086fd8fef7d9e8f34bc79a5ccc9)

---

## 7.2. Arm Throw Controller

### 7.2.1 Overview

The `arm_throw_node.py` implements the complete pick-and-throw sequence for the wx250s arm. It spawns a ball at a known reachable position, moves the arm to the pickup pose using hardcoded joint angles derived from a physical calibration experiment, closes the gripper, lifts the arm, winds back, and executes a fast throw.

**Source:** `ros2_ws/locobot_nodes/locobot_nodes/arm_throw_node.py`

### 7.2.2 wx250s Forward Kinematics

All joint angle computation is grounded in the exact wx250s URDF geometry. The full TF chain from `base_footprint` to the gripper tip:

| Joint | Parent Link | X Offset (m) | Z Offset (m) | Type |
|-------|-------------|--------------|--------------|------|
| base_joint | base_footprint | 0 | 0.0102 | fixed |
| plate | base_link | 0 | 0.08823 | fixed |
| arm_base | plate_link | 0.097277 | 0.0095 | fixed |
| waist | arm_base_link | 0 | 0.066175 | revolute (Z) |
| shoulder (q1) | shoulder_link | 0 | 0.03865 | revolute (Y) |
| elbow (q2) | upper_arm_link | 0.04975 | 0.25 | revolute (Y) |
| forearm_roll | upper_forearm_link | 0.175 | 0 | revolute (X) |
| wrist_angle (q3) | lower_forearm_link | 0.075 | 0 | revolute (Y) |
| wrist_rotate | wrist_link | 0.065 | 0 | revolute (X) |
| ee_arm | gripper_link | 0.043 | 0 | fixed |
| ee_bar | gripper_bar_link | 0.023 | 0 | fixed |
| ee_gripper | fingers_link | 0.027575 | 0 | fixed |

Shoulder joint height from `base_footprint`:

```
SHOULDER_Z = 0.0102 + 0.08823 + 0.0095 + 0.066175 + 0.03865 = 0.212555 m
```

Link lengths used in FK:

| Parameter | Value (m) | Source |
|-----------|-----------|--------|
| UPPER_ARM_X | 0.04975 | elbow joint x offset |
| UPPER_ARM_Z | 0.25 | elbow joint z offset |
| FOREARM_LEN | 0.25 (0.175 + 0.075) | forearm_roll + wrist_angle origins |
| WRIST_TO_TIP | 0.158575 | wrist_rotate + ee_arm + ee_bar + ee_gripper |
| SHOULDER_X | 0.097277 | arm_base_link x offset from plate |
| SHOULDER_Z | 0.212555 | full chain sum from base_footprint |

FK equations in the sagittal plane (X forward, Z up), origin at shoulder joint:

```python
x_elbow = UPPER_ARM_X * cos(q1) - UPPER_ARM_Z * sin(q1)
z_elbow = UPPER_ARM_X * sin(q1) + UPPER_ARM_Z * cos(q1)

x_wrist = x_elbow + FOREARM_LEN * cos(q1 + q2)
z_wrist = z_elbow + FOREARM_LEN * sin(q1 + q2)

x_grip  = x_wrist + WRIST_TO_TIP * cos(q1 + q2 + q3)
z_grip  = z_wrist + WRIST_TO_TIP * sin(q1 + q2 + q3)
```

### 7.2.3 Calibrated Pickup Angles

Pickup joint angles were determined experimentally using `joint_state_publisher_gui` sliders with the `ee_logger` node running in parallel. The arm was jogged to the ball position visually in Gazebo and the exact angles were recorded:

| Joint | Angle (rad) | Angle (deg) | Method |
|-------|-------------|-------------|--------|
| shoulder (q1) | 0.6704 | 38.4° | GUI calibration |
| elbow (q2) | -0.0007 | ~0° | GUI calibration |
| wrist (q3) | -0.0001 | ~0° | GUI calibration |

End effector position at these angles (from `ee_logger` output):

| Position | Value | Frame |
|----------|-------|-------|
| X (forward) | 0.3013 m | base_footprint |
| Y (lateral) | 0.0000 m | base_footprint |
| Z (height) | 0.6932 m | base_footprint |
| Gripper x (rel shoulder) | +0.2040 m | shoulder joint |
| Gripper z (rel shoulder) | +0.4804 m | shoulder joint |
| Reach from shoulder | 0.5219 m | shoulder joint |

The ball is spawned at exactly `X=0.3013, Z=0.6932` from `base_footprint` so the gripper is already at the ball when the arm reaches the pickup pose.

### 7.2.4 Throw Physics

The elbow angle during the throw is computed from target distance using a linear mapping:

```python
t = (distance - THROW_DIST_MIN) / (THROW_DIST_MAX - THROW_DIST_MIN)
throw_elbow = THROW_ELBOW_MIN + t * (THROW_ELBOW_MAX - THROW_ELBOW_MIN)

# THROW_ELBOW_MIN = -1.8 rad
# THROW_ELBOW_MAX = -0.3 rad
# Default target = 3.5 m → elbow = -0.80 rad (-45.8°)
```

Throw parameters:

| Parameter | Value | Description |
|-----------|-------|-------------|
| SHOULDER_WINDUP | -1.2 rad | Arm pulled back for wind-up |
| SHOULDER_THROW | 1.5 rad | Arm swept forward for release |
| wind_back_time | 2.0 s | Hold wind-back position |
| throw_speed_time | 0.4 s | Time after throw before home |
| Gripper release | 0.15 s after throw cmd | Open at peak arm velocity |
| Default target | 3.5 m | Maps to elbow = -0.80 rad |

### 7.2.5 Complete Sequence

| Step | Action | Duration |
|------|--------|----------|
| 1 | Wait for Gazebo to settle | 2.0 s |
| 2 | Spawn ball at exact EE position + publish RViz marker | 0.5 s |
| 3 | Countdown wait | 5.0 s |
| 4 | Open gripper slowly (0.015 → 0.037 m) | 1.0 s |
| 5 | Move arm to pickup pose q1=0.6704, q2=0, q3=0 (30 interpolated steps) | 4.0 s |
| 6 | Close gripper tight (0.037 → 0.016 m) | 2.0 s |
| 7 | Lift arm to home position (slow) | 2.5 s |
| 8 | Wind-back shoulder to -1.2 rad | 2.0 s |
| 9 | THROW — shoulder sweeps to 1.5 rad | 0.15 s |
| 10 | Open gripper — release ball | 0.4 s |
| 11 | Return arm to home | instant |

Slow arm movements use linear interpolation over 30 steps:

```python
for i in range(1, steps+1):
    t = i / steps
    q1_cmd = cur_q1 + t * (target_q1 - cur_q1)
    q2_cmd = cur_q2 + t * (target_q2 - cur_q2)
    q3_cmd = cur_q3 + t * (target_q3 - cur_q3)
```

### 7.2.6 RViz Ball Marker

The `arm_throw_node` publishes a red sphere marker to `/visualization_marker_array` at the ball spawn position for real-time visualization and debugging in RViz2. The marker refreshes every 2 seconds to stay visible throughout the full sequence.

| Marker | Type | Color | Topic |
|--------|------|-------|-------|
| Ball sphere | SPHERE (r=0.02m) | Red (1.0, 0.2, 0.2) | /visualization_marker_array |
| Ball label | TEXT_VIEW_FACING | White | /visualization_marker_array |

---

## 7.3. End Effector Logger

### 7.3.1 Overview

The `ee_logger` node subscribes to `/joint_states` and continuously computes the end effector world position using the wx250s FK equations. It logs joint angles, intermediate link positions, and the gripper tip position whenever any joint changes by more than a configurable threshold (default 0.01 rad).

**Source:** `ros2_ws/locobot_nodes/locobot_nodes/ee_logger.py`

### 7.3.2 Purpose

This node was created specifically to support the arm calibration workflow. By running `ee_logger` alongside `joint_state_publisher_gui`, the engineer can move arm sliders visually in Gazebo and immediately read off exact EE coordinates — including copy-paste ready joint angle constants for `arm_throw_node.py`.

### 7.3.3 Sample Output

```
──────────────────────────────────────────────────────────────────
JOINT ANGLES:
  shoulder(q1) = +0.6704 rad  (+38.4°)
  elbow(q2)    = -0.0007 rad  (-0.0°)
  wrist(q3)    = -0.0001 rad  (-0.0°)

FK POSITIONS (relative to shoulder):
  elbow:   x=-0.1163m  z=+0.2268m
  wrist:   x=+0.0797m  z=+0.3820m
  gripper: x=+0.2040m  z=+0.4804m
  reach from shoulder: 0.5219m

END EFFECTOR (from base_footprint, robot at origin):
  X = +0.3013 m  (forward)
  Y = +0.0000 m  (lateral)
  Z = +0.6932 m  (height)

COPY-PASTE FOR arm_throw_node.py:
  pick_q1 = 0.6704  # shoulder 38.4°
  pick_q2 = -0.0007  # elbow   ~0°
  pick_q3 = -0.0001  # wrist   ~0°
──────────────────────────────────────────────────────────────────
```

### 7.3.4 Joint Name Handling

The node handles both namespaced (`locobot/shoulder`) and plain (`shoulder`) joint names from the `JointState` message, making it compatible with both `joint_state_publisher_gui` and Gazebo bridge outputs.

---

## 7.4. Joint State to Gazebo Bridge

### 7.4.1 Overview

The `joint_state_to_gz` node bridges ROS2 `/joint_states` messages from `joint_state_publisher_gui` to Gazebo's gz transport joint position controller topics. This allows the GUI sliders to directly drive the arm joints in the Gazebo simulation.

**Source:** `ros2_ws/locobot_nodes/locobot_nodes/joint_state_to_gz.py`

### 7.4.2 Topic Mapping

| ROS Joint Name | Gazebo gz Transport Topic |
|----------------|---------------------------|
| waist | /model/locobot/joint/waist/0/cmd_pos |
| shoulder | /model/locobot/joint/shoulder/0/cmd_pos |
| elbow | /model/locobot/joint/elbow/0/cmd_pos |
| forearm_roll | /model/locobot/joint/forearm_roll/0/cmd_pos |
| wrist_angle | /model/locobot/joint/wrist_angle/0/cmd_pos |
| wrist_rotate | /model/locobot/joint/wrist_rotate/0/cmd_pos |
| left_finger | /model/locobot/joint/left_finger/0/cmd_pos |
| right_finger | /model/locobot/joint/right_finger/0/cmd_pos |

The node strips the `locobot/` namespace prefix from joint names before lookup, making it compatible with both namespaced and plain joint name formats.

---

## 7.5. Launch File Integration

### 7.5.1 locobot_gazebo.launch.py

The main Gazebo launch file was updated to include `joint_state_publisher_gui` and `joint_state_to_gz`. Both start with a 10-second `TimerAction` delay to allow the robot to fully spawn before the GUI connects.

| Node | Delay | Purpose |
|------|-------|---------|
| robot_state_publisher | none | Publishes /robot_description and TF |
| ros_gz_sim (Gazebo) | none | Simulation environment |
| ros_gz_sim create | 5s | Spawns robot at (-4, -4, 0.15) |
| ros_gz_bridge | none | Bridges clock, cmd_vel, odom, tf |
| rviz2 | none | Visualization |
| joint_state_publisher_gui | 10s | Slider GUI for manual arm control |
| joint_state_to_gz | 10s | Bridges GUI sliders to Gazebo joints |

### 7.5.2 Calibration Workflow

1. Launch Gazebo: `ros2 launch locobot_gazebo locobot_gazebo.launch.py`
2. Run EE logger: `ros2 run locobot_nodes ee_logger`
3. Move GUI sliders to desired arm position — arm moves in Gazebo
4. Read `COPY-PASTE` block from `ee_logger` terminal
5. Paste `pick_q1/q2/q3` and ball position into `arm_throw_node.py`

---

## 7.6. Individual Contribution Summary

| File | Role |
|------|------|
| `arm_throw_node.py` | FK-based arm controller: ball spawn, slow pickup, throw sequence, RViz marker |
| `ee_logger.py` | FK end effector logger: joint angle monitoring, copy-paste calibration output |
| `joint_state_to_gz.py` | GUI-to-Gazebo bridge: forwards joint_state_publisher_gui to gz transport |
| `locobot_gazebo.launch.py` | Updated launch: added GUI + bridge with 10s delay |

| Commit | Description |
|--------|-------------|
| [`8244a94`](https://github.com/Varad1722/Mobile_Robotics/commit/8244a947a0355086fd8fef7d9e8f34bc79a5ccc9) | Add arm_throw_node.py, ee_logger.py, joint_state_to_gz.py |

---

## 7. Status

| Item | Status | Notes |
|------|--------|-------|
| FK equations (wx250s) | ✅ Complete | Exact URDF geometry used |
| Ball spawn at EE position | ✅ Complete | X=0.3013m, Z=0.6932m |
| Calibrated pickup angles | ✅ Complete | q1=0.6704, q2=-0.0007, q3=-0.0001 |
| Slow arm interpolation | ✅ Complete | 30 steps over 4 seconds |
| Gripper control | ✅ Complete | 0.037m open, 0.016m tight grip |
| Throw sequence | ✅ Complete | Wind-back + fast sweep + release |
| EE logger node | ✅ Complete | Logs on any joint change >0.01 rad |
| GUI-to-Gazebo bridge | ✅ Complete | All 8 joints bridged |
| RViz ball marker | ✅ Complete | Refreshes every 2s |
| End-to-end pick+throw | 🔄 In Progress | Pending physical ball collision tuning |


## 7. Individual Contribution

| Team Member | Primary Technical Role | Key Commits | Files |
|---|---|---|---|
| Dhiren Makwana | Navigation, Detection, Alignment, Integration | [`213c91f`](https://github.com/Varad1722/Mobile_Robotics/commit/213c91f) [`7d5a0b8`](https://github.com/Varad1722/Mobile_Robotics/commit/7d5a0b8) [`c497bac`](https://github.com/Varad1722/Mobile_Robotics/commit/c497bac) [`f342caa`](https://github.com/Varad1722/Mobile_Robotics/commit/f342caa) [`84983b5`](https://github.com/Varad1722/Mobile_Robotics/commit/84983b5) | [`auto_navigator.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/auto_navigator.py) [`base_alignment.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/base_alignment.py) [`ball_detection.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/ball_detection.py) [`locobot_gazebo.launch.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_gazebo/launch/locobot_gazebo.launch.py) |
| Varad Jahagirdar | Perception, Localization, Arm Grasp | [`cfed6b9`](https://github.com/Varad1722/Mobile_Robotics/commit/cfed6b9) [`bab74f0`](https://github.com/Varad1722/Mobile_Robotics/commit/bab74f0) [`bc905a7`](https://github.com/Varad1722/Mobile_Robotics/commit/bc905a7) | [`beacon_localization.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/beacon_localization.py) [`arm_grasp.py`](https://github.com/Varad1722/Mobile_Robotics/blob/Dhiren/ros2_ws/locobot_nodes/locobot_nodes/arm_grasp.py) |
| Sharat Mylavarapu | Throw Controller, End-Effector Logging, Joint State Publishing | [`8244a94`](https://github.com/Varad1722/Mobile_Robotics/commit/8244a947a0355086fd8fef7d9e8f34bc79a5ccc9) | [`arm_throw_node.py`](https://github.com/Varad1722/Mobile_Robotics/commit/8244a947a0355086fd8fef7d9e8f34bc79a5ccc9#diff-c893fe71c85f012012406d04d47af7348b311d2ea1ac73f2c8a866077b3bf7fb) [`ee_logger.py`](https://github.com/Varad1722/Mobile_Robotics/commit/8244a947a0355086fd8fef7d9e8f34bc79a5ccc9) [`joint_state_publisher.py`](https://github.com/Varad1722/Mobile_Robotics/commit/8244a947a0355086fd8fef7d9e8f34bc79a5ccc9#diff-9c44d48a6f7dfb9f0e55fe369e629ce55a56a9f44814b7429331457738318d2eR1-R79) |
