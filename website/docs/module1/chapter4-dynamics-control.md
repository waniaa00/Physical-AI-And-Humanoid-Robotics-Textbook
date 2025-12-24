---
sidebar_position: 4
title: "Chapter 4: Dynamics and Control"
interests: ["dynamics-control"]
---

# Chapter 4: Robot Dynamics and Control

**Week 4 | Module 1: Foundations**

:::info Learning Objectives
- ✅ Understand robot dynamics (forces → motion)
- ✅ Implement PID controllers
- ✅ Design trajectory planners
- ✅ Build impedance controllers for safe interaction
:::

---

## 4.1 Robot Dynamics: The Equation of Motion

### Newton-Euler vs Lagrangian

**Robot Dynamics Equation**:

```
τ = M(q)·q̈ + C(q, q̇)·q̇ + G(q)
```

Where:
- τ: Joint torques (what motors produce)
- M(q): Inertia matrix
- C(q, q̇): Coriolis/centrifugal forces
- G(q): Gravity forces

### Example: 1-Link Pendulum

```
τ = mL²·θ̈ + mgL·sin(θ)
```

**Python Simulation**:
```python
import numpy as np
import matplotlib.pyplot as plt

def simulate_pendulum(m, L, g, theta0, dt=0.01, T=5.0):
    """Simulate 1-link pendulum under gravity."""
    theta = theta0
    theta_dot = 0.0
    time = np.arange(0, T, dt)
    trajectory = []

    for t in time:
        # Dynamics: τ = 0 (no actuation)
        theta_ddot = -(g / L) * np.sin(theta)

        # Integrate (Euler method)
        theta_dot += theta_ddot * dt
        theta += theta_dot * dt

        trajectory.append(theta)

    return time, np.array(trajectory)

# Simulate
time, theta = simulate_pendulum(m=1.0, L=1.0, g=9.81, theta0=np.pi/4)
plt.plot(time, np.degrees(theta))
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Pendulum Motion')
plt.show()
```

---

## 4.2 PID Control

### The PID Controller

```
τ(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt
```

Where:
- e(t) = θ_target - θ_current (error)
- Kp: Proportional gain (stiffness)
- Ki: Integral gain (eliminate steady-state error)
- Kd: Derivative gain (damping)

### Tuning Rules (Ziegler-Nichols)

1. Start with Ki = Kd = 0
2. Increase Kp until oscillations appear → Kp = Ku
3. Measure oscillation period Tu
4. Set: Kp = 0.6×Ku, Ki = 1.2×Ku/Tu, Kd = 0.075×Ku×Tu

### Python PID Implementation

```python
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, target, current):
        """Compute PID control output."""
        error = target - current

        # Proportional term
        P = self.Kp * error

        # Integral term (with anti-windup)
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -10, 10)  # Limit
        I = self.Ki * self.integral

        # Derivative term
        D = self.Kd * (error - self.previous_error) / self.dt
        self.previous_error = error

        return P + I + D

# Example: Position control
pid = PIDController(Kp=10.0, Ki=1.0, Kd=2.0, dt=0.01)

theta_target = np.pi / 2  # 90 degrees
theta_current = 0.0
theta_dot = 0.0

for _ in range(500):  # 5 seconds
    tau = pid.compute(theta_target, theta_current)

    # Simple dynamics: τ = I*α (no gravity)
    theta_ddot = tau / 1.0  # Inertia = 1.0

    # Integrate
    theta_dot += theta_ddot * 0.01
    theta_current += theta_dot * 0.01

    print(f"θ: {np.degrees(theta_current):.2f}°, τ: {tau:.2f}")
```

---

## 4.3 Trajectory Planning

### Point-to-Point Motion

**Cubic Polynomial**:

```
q(t) = a₀ + a₁·t + a₂·t² + a₃·t³
```

Constraints:
- q(0) = q₀ (start position)
- q(T) = qf (end position)
- q̇(0) = 0 (start at rest)
- q̇(T) = 0 (end at rest)

**Python Implementation**:
```python
def cubic_trajectory(q0, qf, T, dt=0.01):
    """Generate cubic polynomial trajectory."""
    # Solve for coefficients
    a0 = q0
    a1 = 0  # Start at rest
    a2 = (3 / T**2) * (qf - q0)
    a3 = -(2 / T**3) * (qf - q0)

    time = np.arange(0, T, dt)
    trajectory = []

    for t in time:
        q = a0 + a1*t + a2*t**2 + a3*t**3
        q_dot = a1 + 2*a2*t + 3*a3*t**2
        q_ddot = 2*a2 + 6*a3*t
        trajectory.append([q, q_dot, q_ddot])

    return time, np.array(trajectory)

# Example: Move from 0° to 90° in 2 seconds
time, traj = cubic_trajectory(0, np.pi/2, T=2.0)
plt.plot(time, np.degrees(traj[:, 0]), label='Position')
plt.plot(time, traj[:, 1], label='Velocity')
plt.legend()
plt.show()
```

### Minimum Jerk Trajectory

Smoother motion (5th-order polynomial):

```python
def minimum_jerk_trajectory(q0, qf, T, dt=0.01):
    """Generate minimum jerk trajectory (5th-order)."""
    time = np.arange(0, T, dt)
    trajectory = []

    for t in time:
        s = t / T  # Normalized time [0, 1]
        # 5th-order polynomial
        q = q0 + (qf - q0) * (10*s**3 - 15*s**4 + 6*s**5)
        trajectory.append(q)

    return time, np.array(trajectory)
```

---

## 4.4 Impedance Control

### Why Impedance Control?

**Problem**: Robots interacting with environment need compliance

**Solution**: Control relationship between force and motion:

```
M·ẍ + B·ẋ + K·x = F_ext
```

Like a mass-spring-damper system!

### Python Implementation

```python
class ImpedanceController:
    def __init__(self, M_desired, B_desired, K_desired):
        self.M_d = M_desired  # Virtual mass
        self.B_d = B_desired  # Damping
        self.K_d = K_desired  # Stiffness

    def compute_torque(self, x_target, x_current, x_dot, F_measured):
        """Compute joint torque for impedance control."""
        # Position error
        x_error = x_target - x_current

        # Desired acceleration
        x_ddot_desired = (F_measured - self.B_d * x_dot - self.K_d * x_error) / self.M_d

        # Convert to joint torque (simplified)
        tau = self.M_d * x_ddot_desired

        return tau
```

### ROS 2 Integration

```python
import rclpy
from geometry_msgs.msg import WrenchStamped

class ImpedanceControllerNode(Node):
    def __init__(self):
        super().__init__('impedance_controller')

        # Subscribe to force sensor
        self.force_sub = self.create_subscription(
            WrenchStamped, '/force_sensor', self.force_callback, 10)

        # Publish joint commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)

        # Controller parameters
        self.controller = ImpedanceController(
            M_desired=1.0, B_desired=10.0, K_desired=100.0)

    def force_callback(self, msg):
        F_measured = msg.wrench.force.z  # Z-axis force
        tau = self.controller.compute_torque(
            x_target=0.5, x_current=self.current_position,
            x_dot=self.current_velocity, F_measured=F_measured)

        # Publish torque command
        cmd = Float64MultiArray()
        cmd.data = [tau]
        self.cmd_pub.publish(cmd)
```

---

## 4.5 Assignment: Week 4

### Project: PID-Controlled Trajectory Planner

**Scenario**: 2-DOF robot arm picks up object

**Requirements**:

1. **Trajectory Planner** (40%)
   - Cubic polynomial from start to grasp pose
   - Duration: 3 seconds
   - Output: Position, velocity, acceleration at 100 Hz

2. **PID Controller** (40%)
   - Tune gains for both joints
   - Track trajectory with error under 2 degrees
   - Implement anti-windup

3. **Gazebo Simulation** (20%)
   - Spawn 2-DOF arm in Gazebo
   - Visualize trajectory in RViz
   - Record joint angles vs time

**Deliverables**:
- `trajectory_planner.py`
- `pid_controller.py`
- Gazebo launch file
- Report: Tracking error analysis

---

## 4.6 Key Takeaways

✅ **Dynamics**: τ = M·q̈ + C·q̇ + G

✅ **PID**: Simple, effective for position control

✅ **Trajectory Planning**: Smooth motion (cubic/quintic)

✅ **Impedance Control**: Compliant interaction with environment

---

**Next Module**: [Module 2: Simulation & Perception →](../module2/chapter5-gazebo-simulation)
