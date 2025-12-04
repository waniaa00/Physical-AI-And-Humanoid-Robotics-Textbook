---
sidebar_position: 1
title: "Chapter 5: Gazebo Simulation"
---

# Chapter 5: Simulation Environments

**Week 5 | Module 2: Simulation & Perception**

:::info Learning Objectives
- ✅ Set up Gazebo and Unity simulation environments
- ✅ Create URDF robot models
- ✅ Integrate ROS 2 with simulators
- ✅ Spawn and control robots in simulation
:::

---

## 5.1 Why Simulation?

### Benefits
- ✅ **Safe testing**: No risk of hardware damage
- ✅ **Fast iteration**: Test algorithms in minutes
- ✅ **Reproducibility**: Exact same conditions every time
- ✅ **Cost**: Free vs $10k-100k for physical robot

---

## 5.2 URDF: Robot Description Format

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
      <origin xyz="0 0 0.25"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
      <origin xyz="0 0 0.25"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

### Visualize URDF

```bash
# Install tools
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-robot-state-publisher

# Launch visualization
ros2 launch urdf_tutorial display.launch.py model:=simple_arm.urdf
```

---

## 5.3 Gazebo Classic Integration

### Launch Robot in Gazebo

**Launch File** (gazebo_spawn.launch.py):
```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'simple_arm',
                '-file', 'simple_arm.urdf',
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': 'simple_arm.urdf'}]
        ),
    ])
```

### Control Joints in Gazebo

```python
import rclpy
from std_msgs.msg import Float64

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.pub = self.create_publisher(
            Float64, '/simple_arm/joint1_position_controller/command', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.angle = 0.0

    def control_loop(self):
        msg = Float64()
        msg.data = self.angle
        self.pub.publish(msg)

        # Sinusoidal motion
        self.angle += 0.01

def main():
    rclpy.init()
    node = JointController()
    rclpy.spin(node)
```

---

## 5.4 Assignment: Week 5

### Project: Create Custom Robot URDF

**Requirements**:
1. 4-DOF manipulator with gripper
2. Include visual meshes (STL files)
3. Proper inertial properties
4. Spawn in Gazebo
5. Control via ROS 2

**Deliverables**:
- `robot.urdf`
- Launch file
- Control script
- Demo video

---

**Next**: [Chapter 6: Isaac Sim →](./chapter6-isaac-sim)
