---
sidebar_position: 2
title: "Chapter 2: ROS 2 Fundamentals"
---

# Chapter 2: ROS 2 Fundamentals

**Week 2 | Module 1: Foundations**

:::info Learning Objectives
- ✅ Understand ROS 2 architecture (nodes, topics, services, actions)
- ✅ Implement publisher/subscriber patterns
- ✅ Create custom message types
- ✅ Build a complete multi-node robot control system
:::

---

## 2.1 ROS 2 Architecture Overview

### The Big Picture

```
┌──────────────────────────────────────────────────┐
│              ROS 2 MIDDLEWARE (DDS)              │
│  (Data Distribution Service - Fast, Reliable)    │
└──────────────────────────────────────────────────┘
         ▲                    ▲                ▲
         │                    │                │
    ┌────┴────┐       ┌───────┴──────┐   ┌────┴─────┐
    │  Node A │◄─────►│   Node B     │◄─►│  Node C  │
    │ Camera  │       │  Controller  │   │  Motors  │
    └─────────┘       └──────────────┘   └──────────┘
```

**Key Concepts**:
- **Node**: A single program that performs computation
- **Topic**: Named bus for streaming data (pub/sub)
- **Service**: Request/response RPC (remote procedure call)
- **Action**: Long-running tasks with feedback

---

## 2.2 Topics: Publisher/Subscriber Pattern

### When to Use Topics
- **Streaming sensor data** (camera, LiDAR, IMU)
- **Continuous control** (motor commands, velocity)
- **Status updates** (battery level, errors)

### Example: Camera Publisher + Image Processor

**Publisher Node** (camera.py):
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 Hz

    def publish_frame(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8).tobytes()

        self.publisher.publish(msg)
        self.get_logger().info('Published camera frame')

def main():
    rclpy.init()
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Subscriber Node** (image_processor.py):
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        # Convert ROS Image to NumPy array
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)

        # Simple processing: compute average brightness
        brightness = np.mean(img)
        self.get_logger().info(f'Average brightness: {brightness:.2f}')

def main():
    rclpy.init()
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Run Both Nodes**:
```bash
# Terminal 1
python3 camera.py

# Terminal 2
python3 image_processor.py
```

---

## 2.3 Services: Request/Response Pattern

### When to Use Services
- **One-time queries** ("Is object detected?")
- **Configuration changes** ("Set motor speed to X")
- **Stateless operations** ("Calculate inverse kinematics")

### Example: Inverse Kinematics Service

**Service Definition** (IKSolve.srv):
```
# Request
geometry_msgs/Pose target_pose

---

# Response
bool success
float64[] joint_angles
string message
```

**Service Server** (ik_server.py):
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Using built-in for demo
import numpy as np

class IKServer(Node):
    def __init__(self):
        super().__init__('ik_server')
        self.srv = self.create_service(
            AddTwoInts, 'solve_ik', self.solve_ik_callback)
        self.get_logger().info('IK Server ready')

    def solve_ik_callback(self, request, response):
        # Simplified: just return random joint angles
        response.sum = request.a + request.b  # Placeholder
        self.get_logger().info(f'IK solved: {response.sum}')
        return response

def main():
    rclpy.init()
    node = IKServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Service Client** (ik_client.py):
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.client = self.create_client(AddTwoInts, 'solve_ik')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK server...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = IKClient()
    response = client.send_request(10, 20)
    client.get_logger().info(f'Result: {response.sum}')
    client.destroy_node()
    rclpy.shutdown()
```

---

## 2.4 Actions: Long-Running Tasks with Feedback

### When to Use Actions
- **Navigation** ("Drive to goal" with progress updates)
- **Grasping** ("Pick object" with force feedback)
- **Trajectory execution** ("Follow path" with waypoint status)

### Example: Navigation Action

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self, Fibonacci, 'navigate_to_goal', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Navigating to goal...')
        feedback_msg = Fibonacci.Feedback()

        # Simulate navigation with progress updates
        for i in range(10):
            feedback_msg.partial_sequence = [i]
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Progress: {i*10}%')
            time.sleep(0.5)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = list(range(10))
        return result

def main():
    rclpy.init()
    node = NavigationActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 2.5 Launch Files: Starting Multiple Nodes

### Why Launch Files?
- Start 10+ nodes with one command
- Set parameters programmatically
- Group related nodes

**Example Launch File** (robot_system.launch.py):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='camera_publisher',
            name='camera',
            parameters=[{'fps': 30}]
        ),
        Node(
            package='my_package',
            executable='image_processor',
            name='processor',
            remappings=[('/camera/image_raw', '/camera/rgb')]
        ),
        Node(
            package='my_package',
            executable='ik_server',
            name='ik_solver'
        ),
    ])
```

**Run Launch File**:
```bash
ros2 launch my_package robot_system.launch.py
```

---

## 2.6 Assignment: Week 2

### Project: Multi-Node Robot Control System

**Scenario**: Build a temperature-controlled autonomous rover.

#### Requirements

**Node 1: Temperature Sensor Simulator**
- Publishes to `/sensors/temperature` (Float32) at 2 Hz
- Simulates random temp: 15-30°C

**Node 2: Thermostat Controller**
- Subscribes to `/sensors/temperature`
- Publishes to `/actuators/fan_speed` (Float32)
  - If temp > 25°C: fan_speed = 100.0
  - If temp < 20°C: fan_speed = 0.0
  - Else: fan_speed = 50.0

**Node 3: Navigation Planner**
- Service `/plan_path` (Request: target_x, target_y | Response: path_length, success)
- Returns random path length (10-100 meters)

**Node 4: Safety Monitor**
- Subscribes to `/sensors/temperature` and `/actuators/fan_speed`
- If temp > 28°C: publish "EMERGENCY STOP" to `/safety/alerts`

**Launch File**: Start all 4 nodes simultaneously

#### Deliverables
1. 4 Python node files
2. 1 launch file
3. README with:
   - ROS 2 graph diagram (use `rqt_graph`)
   - Instructions to run
   - Screenshot of all nodes communicating

---

## 2.7 Key Takeaways

✅ **Topics**: For streaming data (camera, sensors)

✅ **Services**: For request/response (IK, queries)

✅ **Actions**: For long tasks with feedback (navigation, grasping)

✅ **Launch Files**: Start entire robot systems with one command

---

**Next**: [Chapter 3: Robot Kinematics →](./chapter3-kinematics)
