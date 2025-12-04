---
sidebar_position: 1
title: "Chapter 9: Bipedal Locomotion"
---

# Chapter 9: Bipedal Locomotion

**Week 9 | Module 3: Humanoid Robotics**

:::info Learning Objectives
- ✅ Understand Zero Moment Point (ZMP) stability
- ✅ Implement Center of Mass (CoM) trajectory planning
- ✅ Generate walking gaits
- ✅ Make humanoid robot walk in Isaac Sim
:::

---

## 9.1 Why Bipedal Locomotion is Hard

### Static vs Dynamic Stability

**Static Stability** (like tables):
- Center of Mass (CoM) always inside support polygon
- Can stop anytime without falling

**Dynamic Stability** (like humans):
- CoM can be outside support polygon
- Must keep moving to avoid falling

### The Challenge
Humanoid walking = controlled falling!

---

## 9.2 Zero Moment Point (ZMP)

### Definition

**ZMP**: Point on ground where net moment = 0

If ZMP is inside foot support polygon → **Robot is stable** ✅
If ZMP is outside → **Robot will tip over** ❌

### ZMP Equation

The ZMP location is computed using the moment balance equation. For a robot with multiple links, the ZMP position can be calculated from the masses, positions, and accelerations of all links.

Simplified for flat ground walking, the X-coordinate of ZMP is:

```
x_ZMP = Σ(m_i * (x_i * z_ddot_i - z_i * x_ddot_i + z_i * g)) / Σ(m_i * (z_ddot_i + g))
```

Where:
- m_i: Mass of link i
- x_i, z_i: Position of link i
- x_ddot_i, z_ddot_i: Acceleration of link i
- g: Gravity (9.81 m/s²)

### Python Implementation

```python
import numpy as np

def compute_zmp(masses, positions, accelerations, g=9.81):
    """
    Compute ZMP location.

    Args:
        masses: List of link masses [m1, m2, ...]
        positions: List of CoM positions [[x1,y1,z1], ...]
        accelerations: List of CoM accelerations [[ax1,ay1,az1], ...]
        g: Gravity (m/s²)

    Returns:
        [x_zmp, y_zmp]: ZMP coordinates
    """
    positions = np.array(positions)
    accelerations = np.array(accelerations)
    masses = np.array(masses)

    # Numerator and denominator
    numerator_x = np.sum(masses * (positions[:, 0] * (accelerations[:, 2] + g) -
                                     positions[:, 2] * accelerations[:, 0]))
    numerator_y = np.sum(masses * (positions[:, 1] * (accelerations[:, 2] + g) -
                                     positions[:, 2] * accelerations[:, 1]))
    denominator = np.sum(masses * (accelerations[:, 2] + g))

    x_zmp = numerator_x / denominator
    y_zmp = numerator_y / denominator

    return np.array([x_zmp, y_zmp])

# Example: Simple 3-link robot
masses = [5.0, 3.0, 2.0]  # Torso, thigh, shank (kg)
positions = [[0.0, 0.0, 0.8],    # Torso CoM
             [0.1, 0.0, 0.4],    # Thigh CoM
             [0.15, 0.0, 0.1]]   # Shank CoM
accelerations = [[0.0, 0.0, 0.0],
                 [0.5, 0.0, -1.0],
                 [1.0, 0.0, -2.0]]

zmp = compute_zmp(masses, positions, accelerations)
print(f"ZMP: ({zmp[0]:.3f}, {zmp[1]:.3f})")
```

---

## 9.3 CoM Trajectory Planning

### Linear Inverted Pendulum Model (LIPM)

Simplified humanoid dynamics:

```
ẍ = (g/z_c)(x - x_ZMP)
```

Where z_c = constant CoM height

### Walking Gait Generation

**Algorithm**:
1. Define footstep locations
2. Plan ZMP trajectory (inside support polygon)
3. Solve for CoM trajectory using LIPM
4. Compute joint angles via IK

**Python Implementation**:

```python
class WalkingController:
    def __init__(self, step_length=0.2, step_height=0.05, step_duration=0.8):
        self.step_length = step_length
        self.step_height = step_height
        self.step_duration = step_duration
        self.com_height = 0.8  # meters

    def generate_footsteps(self, num_steps):
        """Generate footstep locations."""
        footsteps = []
        for i in range(num_steps):
            x = i * self.step_length
            y = 0.1 if i % 2 == 0 else -0.1  # Alternate left/right
            footsteps.append([x, y, 0.0])
        return np.array(footsteps)

    def plan_com_trajectory(self, footsteps, dt=0.01):
        """Plan CoM trajectory using LIPM."""
        T = len(footsteps) * int(self.step_duration / dt)
        com_traj = np.zeros((T, 3))

        g = 9.81
        omega = np.sqrt(g / self.com_height)

        for i in range(T):
            t = i * dt
            step_idx = int(t / self.step_duration)
            if step_idx >= len(footsteps):
                step_idx = len(footsteps) - 1

            # Target ZMP = current foot position
            zmp_target = footsteps[step_idx]

            # LIPM solution (simplified)
            com_traj[i, 0] = zmp_target[0] + 0.05 * np.cos(omega * t)
            com_traj[i, 1] = zmp_target[1]
            com_traj[i, 2] = self.com_height

        return com_traj

    def swing_foot_trajectory(self, start_pos, end_pos, t, T):
        """Cubic spline for swing foot."""
        s = t / T  # Normalized time [0, 1]

        # Position interpolation
        pos = start_pos + (end_pos - start_pos) * (3*s**2 - 2*s**3)

        # Add clearance height (parabola)
        pos[2] = self.step_height * 4 * s * (1 - s)

        return pos

# Example usage
controller = WalkingController(step_length=0.2, step_duration=0.8)
footsteps = controller.generate_footsteps(num_steps=10)
com_trajectory = controller.plan_com_trajectory(footsteps)

print(f"Generated {len(com_trajectory)} CoM waypoints")
```

---

## 9.4 Full Walking Pipeline

### ROS 2 Walking Controller

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class BipedalWalkingNode(Node):
    def __init__(self):
        super().__init__('bipedal_walking')

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Walking controller
        self.controller = WalkingController()
        self.footsteps = self.controller.generate_footsteps(num_steps=10)
        self.com_traj = self.controller.plan_com_trajectory(self.footsteps)

        # State
        self.step_idx = 0
        self.traj_idx = 0

        # Control loop at 100 Hz
        self.timer = self.create_timer(0.01, self.control_loop)

    def control_loop(self):
        if self.traj_idx >= len(self.com_traj):
            self.get_logger().info('Walking complete!')
            return

        # Current CoM target
        com_target = self.com_traj[self.traj_idx]

        # Compute foot positions
        left_foot, right_foot = self.compute_foot_positions(com_target)

        # Inverse kinematics (simplified)
        joint_angles = self.solve_ik(com_target, left_foot, right_foot)

        # Publish joint commands
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['hip_l', 'knee_l', 'ankle_l', 'hip_r', 'knee_r', 'ankle_r']
        msg.position = joint_angles
        self.joint_pub.publish(msg)

        self.traj_idx += 1

    def solve_ik(self, com_target, left_foot, right_foot):
        """Simplified IK for leg joints."""
        # This would use actual IK solver in practice
        # For demo, return placeholder values
        return [0.0] * 6  # 6 leg joints
```

---

## 9.5 Assignment: Week 9

### Project: ZMP Walking Controller

**Goal**: Make humanoid walk 5 meters in Isaac Sim

**Requirements**:

1. **Footstep Planner** (30%)
   - 10 steps forward
   - 0.2m step length
   - 0.8s per step

2. **CoM Trajectory** (30%)
   - LIPM-based planning
   - ZMP always inside support polygon
   - Verify with ZMP computation

3. **Leg IK** (20%)
   - Analytical IK for 6-DOF legs
   - Account for hip width (0.2m)

4. **Isaac Sim Integration** (20%)
   - Spawn humanoid (use Unitree H1 or similar)
   - Execute walking gait
   - Record ZMP trajectory

**Deliverables**:
- `walking_controller.py`
- Isaac Sim script
- Video: 5-meter walk
- Report: ZMP analysis (plot ZMP vs support polygon)

**Bonus** (+10%): Implement online balance recovery (push robot and it recovers)

---

## 9.6 Key Takeaways

✅ **ZMP**: Must stay inside support polygon for stability

✅ **LIPM**: Simplified model for CoM trajectory planning

✅ **Gait Cycle**: Stance phase (support) + Swing phase (foot in air)

✅ **Humanoid Walking**: Requires continuous balance control

---

**Next**: [Chapter 10: Manipulation and Grasping →](./chapter10-manipulation)
