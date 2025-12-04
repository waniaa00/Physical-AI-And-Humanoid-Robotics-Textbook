---
sidebar_position: 3
title: "Chapter 3: Robot Kinematics"
---

# Chapter 3: Robot Kinematics

**Week 3 | Module 1: Foundations**

:::info Learning Objectives
- ✅ Understand Denavit-Hartenberg (DH) parameters
- ✅ Compute forward kinematics for manipulators
- ✅ Solve inverse kinematics (analytical & numerical)
- ✅ Calculate Jacobians for velocity control
- ✅ Implement FK/IK for a 6-DOF robot arm
:::

---

## 3.1 Why Kinematics Matters

**Problem**: Given joint angles θ₁, θ₂, ..., θₙ, where is the robot's end-effector?

**Applications**:
- 🎯 **Pick-and-place**: "Move gripper to (x, y, z)"
- 🚶 **Walking**: "Place foot at specific location"
- 🤖 **Teleoperation**: "Match human hand position"

---

## 3.2 Coordinate Frames and Transformations

### Homogeneous Transformation Matrix

Represents **position + orientation** in 3D space:

A 4x4 homogeneous transformation matrix has the structure:

```
T = [ R | p ]
    [---+---]
    [ 0 | 1 ]

Where:
R = 3x3 rotation matrix (orientation)
p = 3x1 position vector (translation)

Full form:
T = [ r11  r12  r13  px ]
    [ r21  r22  r23  py ]
    [ r31  r32  r33  pz ]
    [  0    0    0    1 ]
```

Where:
- **R**: 3×3 rotation matrix (orientation)
- **p**: 3×1 position vector (translation)

### Example: 2D Rotation + Translation

Rotate 90° about Z-axis, then translate by (2, 3):

```
T = [ cos(90°)  -sin(90°)   2 ]     [ 0  -1   2 ]
    [ sin(90°)   cos(90°)   3 ]  =  [ 1   0   3 ]
    [    0          0        1 ]     [ 0   0   1 ]
```

**Python Implementation**:
```python
import numpy as np

def transformation_matrix_2d(theta, tx, ty):
    """Create 2D homogeneous transformation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta), tx],
        [np.sin(theta),  np.cos(theta), ty],
        [0,              0,              1]
    ])

# Example: Rotate 90° and translate
T = transformation_matrix_2d(np.pi/2, 2, 3)
print(T)
```

---

## 3.3 Denavit-Hartenberg (DH) Parameters

### The Standard Convention (Craig 1989)

Each joint is described by **4 parameters**:

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| **Joint angle** | θᵢ | Rotation about Zᵢ₋₁ (variable for revolute) |
| **Link offset** | dᵢ | Translation along Zᵢ₋₁ (variable for prismatic) |
| **Link length** | aᵢ | Translation along Xᵢ |
| **Link twist** | αᵢ | Rotation about Xᵢ |

### DH Transformation Matrix

The transformation from frame i-1 to frame i using DH parameters:

```
T(i-1,i) = [ cos(θᵢ)  -sin(θᵢ)cos(αᵢ)   sin(θᵢ)sin(αᵢ)   aᵢcos(θᵢ) ]
           [ sin(θᵢ)   cos(θᵢ)cos(αᵢ)  -cos(θᵢ)sin(αᵢ)   aᵢsin(θᵢ) ]
           [    0          sin(αᵢ)          cos(αᵢ)           dᵢ     ]
           [    0             0                 0              1      ]
```

### Example: 2-Link Planar Arm

```
     Joint 2
        ●
       /
      / L2
     /
    ● Joint 1
    |
    | L1
    |
    ● Base
```

**DH Table**:

| Joint | θᵢ | dᵢ | aᵢ | αᵢ |
|-------|-----|-----|-----|-----|
| 1 | θ₁* | 0 | L₁ | 0 |
| 2 | θ₂* | 0 | L₂ | 0 |

(*variable)

**Python Implementation**:
```python
def dh_matrix(theta, d, a, alpha):
    """Compute DH transformation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),               d],
        [0,              0,                             0,                           1]
    ])

# 2-link arm with L1=1.0, L2=0.8
L1, L2 = 1.0, 0.8
theta1, theta2 = np.pi/4, np.pi/6  # 45° and 30°

T1 = dh_matrix(theta1, 0, L1, 0)  # Base to Joint 1
T2 = dh_matrix(theta2, 0, L2, 0)  # Joint 1 to Joint 2

# End-effector pose
T_end = T1 @ T2  # Matrix multiplication
print(f"End-effector position: ({T_end[0,3]:.3f}, {T_end[1,3]:.3f})")
```

**Output**:
```
End-effector position: (1.400, 1.094)
```

---

## 3.4 Forward Kinematics (FK)

### Problem Statement

**Given**: Joint angles θ = [θ₁, θ₂, ..., θₙ]
**Find**: End-effector pose (position + orientation)

### Algorithm

1. Build DH table for robot
2. Compute T₁, T₂, ..., Tₙ using DH parameters
3. Multiply: T_end = T₁ × T₂ × ... × Tₙ
4. Extract position from T_end[0:3, 3]

### Example: PUMA 560 Robot (6-DOF)

**DH Table** (simplified):

| Joint | θᵢ | dᵢ | aᵢ | αᵢ |
|-------|-----|-----|-----|-----|
| 1 | θ₁* | 0.67 | 0 | 90° |
| 2 | θ₂* | 0 | 0.43 | 0° |
| 3 | θ₃* | 0 | 0.02 | 90° |
| 4 | θ₄* | 0.43 | 0 | -90° |
| 5 | θ₅* | 0 | 0 | 90° |
| 6 | θ₆* | 0.056 | 0 | 0° |

**Python FK Solver**:
```python
class RobotArm:
    def __init__(self, dh_params):
        """
        dh_params: List of (theta, d, a, alpha) for each joint
        """
        self.dh_params = dh_params

    def forward_kinematics(self, joint_angles):
        """Compute FK for given joint angles."""
        T = np.eye(4)  # Start with identity matrix

        for i, theta in enumerate(joint_angles):
            theta_i, d_i, a_i, alpha_i = self.dh_params[i]
            theta_total = theta_i + theta  # Add joint variable
            T_i = dh_matrix(theta_total, d_i, a_i, alpha_i)
            T = T @ T_i  # Accumulate transformations

        return T

# Example: 3-DOF arm
dh_params = [
    (0, 0, 1.0, 0),      # Joint 1
    (0, 0, 0.8, 0),      # Joint 2
    (0, 0, 0.5, 0)       # Joint 3
]

robot = RobotArm(dh_params)
joint_angles = [np.pi/4, np.pi/6, np.pi/3]  # 45°, 30°, 60°
T_end = robot.forward_kinematics(joint_angles)

print(f"End-effector position: {T_end[0:3, 3]}")
print(f"Orientation matrix:\n{T_end[0:3, 0:3]}")
```

---

## 3.5 Inverse Kinematics (IK)

### Problem Statement

**Given**: Desired end-effector pose (x, y, z, R)
**Find**: Joint angles θ = [θ₁, θ₂, ..., θₙ]

### Method 1: Analytical IK (Closed-Form)

**For 2-Link Planar Arm**:

Given target (x_target, y_target), the joint angles can be computed using trigonometry:

```
θ₂ = ±arccos((x² + y² - L₁² - L₂²) / (2·L₁·L₂))

θ₁ = arctan(y/x) - arctan((L₂·sin(θ₂)) / (L₁ + L₂·cos(θ₂)))
```

**Python Implementation**:
```python
def inverse_kinematics_2link(x, y, L1, L2):
    """Analytical IK for 2-link planar arm."""
    # Check if target is reachable
    distance = np.sqrt(x**2 + y**2)
    if distance > (L1 + L2) or distance < abs(L1 - L2):
        return None, None  # Unreachable

    # Elbow-down solution
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(cos_theta2)

    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1, theta2

# Example
L1, L2 = 1.0, 0.8
x_target, y_target = 1.4, 1.0

theta1, theta2 = inverse_kinematics_2link(x_target, y_target, L1, L2)
print(f"Joint angles: θ1={np.degrees(theta1):.2f}°, θ2={np.degrees(theta2):.2f}°")

# Verify with FK
T = robot.forward_kinematics([theta1, theta2])
print(f"Achieved position: ({T[0,3]:.3f}, {T[1,3]:.3f})")
```

### Method 2: Numerical IK (Jacobian-Based)

For complex robots (6+ DOF), use iterative methods:

**Algorithm** (Newton-Raphson):
```
1. Start with initial guess θ₀
2. Repeat until converged:
   a. Compute current pose: T_current = FK(θ)
   b. Compute error: Δx = x_target - T_current
   c. Compute Jacobian: J(θ)
   d. Update: θ_new = θ + J⁻¹ × Δx
```

**Python Implementation**:
```python
def numerical_ik(robot, target_pos, initial_guess, max_iter=100, tol=1e-3):
    """Numerical IK using Jacobian pseudo-inverse."""
    theta = np.array(initial_guess)

    for _ in range(max_iter):
        # Current end-effector position
        T = robot.forward_kinematics(theta)
        current_pos = T[0:3, 3]

        # Position error
        error = target_pos - current_pos
        if np.linalg.norm(error) < tol:
            return theta  # Converged

        # Compute Jacobian (numerical differentiation)
        J = compute_jacobian(robot, theta)

        # Update joint angles
        delta_theta = np.linalg.pinv(J) @ error
        theta += delta_theta * 0.1  # Step size = 0.1

    return theta  # Max iterations reached

def compute_jacobian(robot, theta, epsilon=1e-5):
    """Compute Jacobian using finite differences."""
    J = np.zeros((3, len(theta)))  # 3D position only

    T0 = robot.forward_kinematics(theta)
    p0 = T0[0:3, 3]

    for i in range(len(theta)):
        theta_plus = theta.copy()
        theta_plus[i] += epsilon
        T_plus = robot.forward_kinematics(theta_plus)
        p_plus = T_plus[0:3, 3]

        J[:, i] = (p_plus - p0) / epsilon

    return J
```

---

## 3.6 Jacobian: Velocity Kinematics

### What is the Jacobian?

Maps **joint velocities** to **end-effector velocities**:

```
ẋ = J(θ) · θ̇
```

Where:
- ẋ: End-effector velocity (6×1: linear + angular)
- J(θ): Jacobian matrix (6×n)
- θ̇: Joint velocities (n×1)

### Applications

1. **Velocity Control**: "Move end-effector at 0.1 m/s in X direction"
2. **Singularity Avoidance**: det(J) ≈ 0 means robot is stuck
3. **Force Control**: F = J^T × τ (joint torques to end-effector force)

### Example: 2-Link Jacobian

For a 2-link planar arm, the Jacobian is:

```
J = [ -L₁sin(θ₁) - L₂sin(θ₁+θ₂)    -L₂sin(θ₁+θ₂) ]
    [  L₁cos(θ₁) + L₂cos(θ₁+θ₂)     L₂cos(θ₁+θ₂) ]
```

**Python**:
```python
def jacobian_2link(theta1, theta2, L1, L2):
    """Analytical Jacobian for 2-link arm."""
    J = np.array([
        [-L1*np.sin(theta1) - L2*np.sin(theta1+theta2), -L2*np.sin(theta1+theta2)],
        [ L1*np.cos(theta1) + L2*np.cos(theta1+theta2),  L2*np.cos(theta1+theta2)]
    ])
    return J

# Example: Move end-effector at 0.1 m/s in Y direction
theta1, theta2 = np.pi/4, np.pi/6
J = jacobian_2link(theta1, theta2, 1.0, 0.8)

desired_velocity = np.array([0, 0.1])  # [v_x, v_y]
joint_velocities = np.linalg.pinv(J) @ desired_velocity
print(f"Required joint velocities: {joint_velocities} rad/s")
```

---

## 3.7 ROS 2 Integration

### Using MoveIt 2 for IK

```python
import rclpy
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

class IKClient:
    def __init__(self):
        self.node = rclpy.create_node('ik_client')
        self.client = self.node.create_client(
            GetPositionIK, 'compute_ik')

    def solve_ik(self, x, y, z):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0  # No rotation

        request.ik_request.pose_stamped = pose

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()
```

---

## 3.8 Assignment: Week 3

### Project: FK/IK for 6-DOF Manipulator

**Part 1**: Forward Kinematics (40%)
1. Implement `forward_kinematics()` for PUMA 560
2. Test with 5 different joint configurations
3. Visualize in RViz using `robot_state_publisher`

**Part 2**: Inverse Kinematics (40%)
1. Implement analytical IK for first 3 joints (position)
2. Implement numerical IK for full 6-DOF (position + orientation)
3. Test reachability for 100 random targets

**Part 3**: Jacobian (20%)
1. Compute Jacobian using finite differences
2. Identify singularities (det(J) < 0.01)
3. Implement velocity control: "Move in circle at 0.05 m/s"

**Deliverables**:
- Python module `kinematics.py` with FK, IK, Jacobian
- Test script with unit tests
- Report: FK/IK accuracy analysis

---

## 3.9 Key Takeaways

✅ **DH Parameters**: Standardized way to describe robot geometry

✅ **Forward Kinematics**: Joint angles → End-effector pose

✅ **Inverse Kinematics**: Target pose → Joint angles (multiple solutions!)

✅ **Jacobian**: Maps joint velocities to end-effector velocities

✅ **Singularities**: Configurations where robot loses mobility

---

**Next**: [Chapter 4: Dynamics and Control →](./chapter4-dynamics-control)

Learn how forces and torques affect robot motion, and design controllers.
