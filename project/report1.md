### Custom Modules

#### 🔧 CG Stability Controller

Dynamically adjusts the **manipulator posture** to maintain stable center-of-gravity positioning during base motion.

**Problem:** When the TurtleBot accelerates or decelerates, inertial forces shift the combined center of mass of the base-arm system, reducing traction and destabilizing the robot.

**Solution:** The controller estimates the projected center of gravity and adjusts the arm configuration to keep the CoM within a stable support region reducing load transfer and improving traction consistency at the drive wheels.

**Center of Gravity Estimation:**

$$x_{cg} = \frac{\sum m_i x_i}{\sum m_i}$$

---

#### 🎯 Vacuum Throw Release Controller

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

> By synchronizing release timing with arm motion, the system maximizes throwing **range and repeatability**.

---

<div align="center">

</div>
