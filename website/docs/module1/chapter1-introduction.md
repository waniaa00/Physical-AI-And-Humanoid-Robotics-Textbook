---
sidebar_position: 1
title: "Chapter 1: Introduction to Physical AI"
interests: ["physical-ai", "ros2"]
---

# Chapter 1: What is Physical AI?

**Week 1 | Module 1: Foundations**

:::info Learning Objectives
By the end of this chapter, you will:
- ✅ Understand the difference between Physical AI and traditional robotics
- ✅ Identify key components of embodied AI systems
- ✅ Recognize real-world applications of Physical AI
- ✅ Set up your ROS 2 development environment
:::

---

## 1.1 The Embodied AI Revolution

### What is Physical AI?

**Physical AI** refers to artificial intelligence systems that:
1. **Interact with the physical world** (not just digital data)
2. **Learn from embodied experience** (sensors, actuators, physics)
3. **Make decisions in real-time** (perception → reasoning → action loop)
4. **Adapt to changing environments** (dynamic, unstructured spaces)

###Traditional Robotics vs Physical AI

| Aspect | Traditional Robotics | Physical AI |
|--------|---------------------|-------------|
| **Programming** | Rule-based, explicit instructions | Learning-based, emergent behaviors |
| **Perception** | Pre-programmed object recognition | Self-supervised vision models |
| **Control** | Fixed trajectories, PID loops | Adaptive policies (RL, VLA models) |
| **Environment** | Structured (factory floors) | Unstructured (homes, outdoors) |
| **Task Specification** | Code (e.g., "move to (x, y, z)") | Natural language ("fetch the red cup") |
| **Examples** | Industrial arms, CNC machines | Humanoid robots, autonomous drones |

### Key Insight
> **Physical AI = Robotics + Deep Learning + Embodied Experience**

Traditional robots follow scripts. Physical AI systems **learn from interaction**.

---

## 1.2 Core Components of Physical AI Systems

### The Perception-Planning-Action Loop

```
┌─────────────────────────────────────────────────┐
│                 ENVIRONMENT                     │
│  (Objects, Obstacles, Humans, Dynamic Changes)  │
└──────────────┬──────────────────────────────────┘
               │ Sensor Data
               ▼
       ┌───────────────┐
       │  PERCEPTION   │  ← Cameras, LiDAR, IMU
       │  (See & Feel) │
       └───────┬───────┘
               │ Scene Understanding
               ▼
       ┌───────────────┐
       │   PLANNING    │  ← Task Reasoning (LLMs, VLAs)
       │ (Think & Plan)│
       └───────┬───────┘
               │ Action Commands
               ▼
       ┌───────────────┐
       │    ACTION     │  ← Motors, Grippers, Wheels
       │ (Move & Act)  │
       └───────┬───────┘
               │ Physical Changes
               └──────────► ENVIRONMENT (loop closes)
```

### 1. Perception Systems
**What the robot sees and feels**

- **Vision**: Cameras (RGB, depth, stereo)
  - Object detection (YOLO, Detectron2)
  - 3D pose estimation (FoundationPose)
  - Semantic segmentation (Mask R-CNN)

- **Proprioception**: Internal sensors
  - Joint encoders (motor positions)
  - IMU (acceleration, orientation)
  - Force/torque sensors (touch, pressure)

- **Exteroception**: External sensors
  - LiDAR (distance mapping)
  - Microphones (sound localization)
  - Tactile sensors (surface texture)

**Example: Picking up a cup**
```python
# Perception pipeline
camera_image = robot.get_camera_image()  # RGB image
depth_map = robot.get_depth()            # Distance to objects
detected_objects = yolo_detector(camera_image)  # "cup" at (x, y)
cup_3d_pose = estimate_pose(detected_objects['cup'], depth_map)
```

### 2. Planning Systems
**What the robot thinks and decides**

- **Traditional Planning**: Search algorithms (A*, RRT)
  - Pros: Guaranteed solutions (if they exist)
  - Cons: Requires perfect world model

- **Learning-Based Planning**: Neural networks (VLAs, RL)
  - Pros: Generalizes to novel situations
  - Cons: Requires training data

- **Hybrid Planning**: LLM + Classical planners
  - LLM: High-level task decomposition ("make coffee" → steps)
  - Classical: Low-level motion planning (collision-free paths)

**Example: LLM-based task planning**
```python
# User command (natural language)
command = "Fetch the red cup from the table"

# LLM breaks down task
llm_output = gpt4.plan(command)
# Output: [
#   "navigate to table",
#   "detect red cup",
#   "plan grasp",
#   "execute pick",
#   "navigate to user",
#   "hand over cup"
# ]

# Execute each subtask with specialized controllers
for subtask in llm_output:
    robot.execute(subtask)
```

### 3. Action Systems
**What the robot actually does**

- **Manipulation**: Arms, grippers
  - Trajectory execution (joint space, task space)
  - Force control (compliant grasping)

- **Locomotion**: Wheels, legs
  - Point-to-point navigation
  - Dynamic walking (for bipeds)

- **Actuation**: Motors
  - Position control (PID)
  - Torque control (force-sensitive tasks)

**Example: Grasping with force control**
```python
# Close gripper until force threshold reached
while gripper.force < GRASP_FORCE_THRESHOLD:
    gripper.close(speed=0.01)  # Slow, controlled closure
    time.sleep(0.01)

# Lift object
arm.move_to_pose(lift_height=0.2)
```

---

## 1.3 Real-World Applications

### 🏭 Manufacturing (Already Deployed)
- **Tesla Optimus**: Humanoid for factory automation
- **Boston Dynamics Spot**: Warehouse inspection
- **ABB Yumi**: Collaborative assembly arms

**Task Example**: Pick-and-place 1,000 parts/hour with 99.9% accuracy

### 🏥 Healthcare (Emerging)
- **Intuitive Surgical da Vinci**: Teleoperated surgery
- **Diligent Robotics Moxi**: Hospital delivery robot
- **Toyota HSR**: Home care assistance

**Task Example**: Deliver medications to 50 patient rooms autonomously

### 🏠 Home Robotics (Research → Production)
- **Hello Robot Stretch**: Mobile manipulator
- **Amazon Astro**: Home security and monitoring
- **Figure 01**: General-purpose humanoid

**Task Example**: "Clean the kitchen" (navigation + manipulation + object detection)

### 🚗 Autonomous Vehicles (Rapid Growth)
- **Waymo**: Self-driving taxis (100k+ miles/week)
- **Tesla FSD**: Consumer autonomous driving
- **Nuro**: Autonomous delivery robots

**Task Example**: Navigate city streets, avoid pedestrians, park autonomously

---

## 1.4 Why Humanoid Robots?

### The Case for Human-Shaped Robots

**Problem**: Our world is designed for humans
- Door handles at human height
- Stairs, not ramps
- Tools shaped for human hands

**Solution**: Build robots with human form factor
- Two arms for manipulation
- Two legs for stair climbing
- Human height for reachability

### Technical Challenges
1. **Balance**: Bipedal locomotion is dynamically unstable
2. **Control**: 30+ degrees of freedom (joints)
3. **Power**: Battery life vs. weight tradeoff
4. **Cost**: Complex mechanisms are expensive

### Why Now?
Three breakthroughs enable modern humanoids:

1. **AI Models**: VLAs (RT-2, OpenVLA) for general manipulation
2. **Simulation**: Isaac Sim, MuJoCo for physics-accurate training
3. **Hardware**: Cheaper actuators, better batteries, edge AI (Jetson)

---

## 1.5 The Physical AI Stack

### Layer 1: Hardware (Bottom)
- Sensors: Cameras, LiDAR, IMU
- Actuators: Motors, servos, hydraulics
- Compute: CPU/GPU (Jetson Orin, Intel NUC)

### Layer 2: Middleware (ROS 2)
- Communication: Topics, services, actions
- Drivers: Camera, motor, sensor interfaces
- Tools: RViz (visualization), rqt (debugging)

### Layer 3: Perception
- Vision: Object detection, segmentation
- Mapping: SLAM (Cartographer, ORB-SLAM3)
- Localization: GPS, IMU fusion

### Layer 4: Planning
- Motion Planning: MoveIt 2, OMPL
- Task Planning: LLMs (GPT-4, Claude)
- Learning: VLAs (RT-2, Octo, OpenVLA)

### Layer 5: Applications (Top)
- High-level tasks: "Make coffee", "Clean room"
- User interfaces: Voice, gestures, apps

---

## 1.6 Hands-On: Setup Your Environment

### Prerequisites
- Ubuntu 22.04 LTS (or Docker Desktop on Windows/Mac)
- 50GB free disk space
- Internet connection

### Step 1: Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete -y

# Install colcon (build tool)
sudo apt install python3-colcon-common-extensions -y

# Source ROS 2 (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Step 2: Verify Installation

```bash
# Check ROS 2 version
ros2 --version
# Expected: ros2 cli version: 0.25.x

# Test talker/listener
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2 (new terminal)
ros2 run demo_nodes_cpp listener
```

**Expected Output**:
```
Terminal 1: [INFO] [talker]: Publishing: 'Hello World: 1'
Terminal 2: [INFO] [listener]: I heard: [Hello World: 1]
```

### Step 3: Install VS Code with ROS Extensions

```bash
# Install VS Code
sudo snap install --classic code

# Install ROS extension (run in VS Code)
# Extensions → Search "ROS" → Install "ROS" by Microsoft
```

### Step 4: Create Your First Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build

# Source workspace
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 1.7 Your First ROS 2 Node

### Create a Simple Publisher

```python
# ~/ros2_ws/src/my_first_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Physical AI! Count: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldPublisher()
    rclpy.spin(node)  # Keep node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Run Your Node

```bash
# Make executable
chmod +x ~/ros2_ws/src/my_first_node.py

# Run node
python3 ~/ros2_ws/src/my_first_node.py
```

**Expected Output**:
```
[INFO] [hello_world_publisher]: Publishing: "Hello Physical AI! Count: 0"
[INFO] [hello_world_publisher]: Publishing: "Hello Physical AI! Count: 1"
[INFO] [hello_world_publisher]: Publishing: "Hello Physical AI! Count: 2"
...
```

### Visualize with `ros2 topic`

```bash
# List active topics
ros2 topic list
# Output: /hello_topic

# Echo topic messages
ros2 topic echo /hello_topic
```

---

## 1.8 Assignment: Week 1

### Part 1: Conceptual Questions (30 minutes)

1. **Comparison Table**: Fill out a table comparing Traditional Robotics vs Physical AI systems for:
   - Industrial pick-and-place
   - Home cooking robot
   - Autonomous car

2. **System Design**: For a "Fetch me a drink" robot, list:
   - Required sensors (with justification)
   - Planning steps (high-level task breakdown)
   - Actuators needed

### Part 2: Practical Implementation (90 minutes)

**Goal**: Create a ROS 2 system with 3 nodes

1. **Temperature Sensor Node** (Publisher)
   - Publishes random temperature (15-30°C) every 2 seconds
   - Topic: `/temperature`

2. **Thermostat Controller Node** (Subscriber + Publisher)
   - Subscribes to `/temperature`
   - If temp > 25°C: publish "AC ON" to `/ac_command`
   - If temp < 20°C: publish "HEATER ON" to `/ac_command`
   - Else: publish "STANDBY"

3. **Logger Node** (Subscriber)
   - Subscribes to both `/temperature` and `/ac_command`
   - Prints formatted log: `[TIME] Temp: 28°C | AC: ON`

**Deliverables**:
- 3 Python files (one per node)
- `README.md` with run instructions
- Screenshot of all 3 nodes running

### Submission
- GitHub repository: `physical-ai-week1`
- Include `requirements.txt` for dependencies

---

## 1.9 Additional Resources

### Documentation
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

### Videos
- [Boston Dynamics Atlas](https://www.youtube.com/watch?v=tF4DML7FIWk) (humanoid demo)
- [Figure 01 Demo](https://www.figure.ai/) (embodied AI)

### Papers
- **RT-2**: *RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control* (Brohan et al., 2023)
- **Physical AI Vision**: *On the Opportunities and Risks of Foundation Models* (Bommasani et al., 2021)

---

## 1.10 Key Takeaways

✅ **Physical AI** = AI systems that interact with the physical world through sensors and actuators

✅ **Core Loop**: Perception (sense) → Planning (think) → Action (do) → repeat

✅ **ROS 2** is the middleware connecting sensors, planning, and actuators

✅ **Humanoid robots** are ideal for human-designed environments but technically challenging

✅ **You just built your first ROS 2 node!** 🎉

---

**Next Chapter**: [Chapter 2: ROS 2 Fundamentals →](./chapter2-ros2-fundamentals)

Learn about topics, services, actions, and build a complete multi-node robot control system.
