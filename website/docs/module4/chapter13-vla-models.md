---
sidebar_position: 1
title: "Chapter 13: Vision-Language-Action Models"
---

# Chapter 13: Vision-Language-Action Models

**Week 13 | Module 4: AI Integration**

:::info Learning Objectives
- ✅ Understand VLA architecture (RT-1, RT-2, OpenVLA)
- ✅ Deploy OpenVLA for robotic manipulation
- ✅ Fine-tune VLA on custom tasks
- ✅ Integrate VLA with ROS 2
:::

---

## 13.1 What are VLAs?

### Traditional vs VLA Approach

**Traditional Robotics**:
```
Task: "Pick up red cup"
↓
1. Write object detection code
2. Write grasp planning code
3. Write motion planning code
4. Write execution code
```
**Takes weeks for one task** ⏱️

**VLA Approach**:
```
Task: "Pick up red cup"
↓
VLA Model (pre-trained on 800k tasks)
↓
Actions: [joint_positions_t1, joint_positions_t2, ...]
```
**Works immediately** ✅

---

## 13.2 VLA Model Architectures

### RT-1: Robotics Transformer (Google, 2022)

**Architecture**:
```
Image (300x300) → Vision Encoder (EfficientNet)
        ↓
  Token Embeddings (81 tokens)
        ↓
  Transformer (8 layers, 124M params)
        ↓
  Action Tokens (7-DOF actions)
```

**Training Data**: 130k real robot demonstrations

**Performance**:
- 97% success on seen tasks
- 76% success on novel instructions
- 3 Hz inference

### RT-2: Vision-Language-Action (Google, 2023)

**Key Innovation**: Web-scale vision-language pretraining

**Architecture**:
```
Image + Text → PaLI-X (55B params)
        ↓
  Vision-Language Embeddings
        ↓
  Action Head (7-DOF + gripper)
```

**Training**:
1. Pre-train on web images + captions (WebLI, 10B images)
2. Fine-tune on robot data (130k demos)

**Results**:
- 62% better generalization than RT-1
- Understands abstract concepts ("pick the extinct animal")

### OpenVLA: Open-Source VLA (Stanford, 2024)

**Why OpenVLA?**
- ✅ Fully open weights (7B params)
- ✅ Apache 2.0 license (commercial use OK)
- ✅ Trained on Open-X Embodiment (800k trajectories, 22 robots)

**Architecture**:
```
Image (224x224) → CLIP ViT-L/14
Text → LLaMA 2 (7B)
        ↓
  Fused Embeddings
        ↓
  MLP Action Head (7-DOF)
```

---

## 13.3 Deploy OpenVLA

### Installation

```bash
# Install dependencies
pip install torch torchvision transformers
pip install open-vla  # Simplified, actual: from source

# Download model weights
from transformers import AutoModelForVision2Seq, AutoProcessor

model = AutoModelForVision2Seq.from_pretrained("openvla/openvla-7b")
processor = AutoProcessor.from_pretrained("openvla/openvla-7b")
```

### Inference

```python
import torch
from PIL import Image
import numpy as np

class OpenVLAController:
    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = AutoModelForVision2Seq.from_pretrained(
            "openvla/openvla-7b").to(self.device)
        self.processor = AutoProcessor.from_pretrained("openvla/openvla-7b")

    def predict_action(self, image, instruction):
        """
        Predict robot action from image and language instruction.

        Args:
            image: PIL Image or numpy array (H, W, 3)
            instruction: str, e.g., "pick up the red cup"

        Returns:
            action: np.array (7,) - [x, y, z, roll, pitch, yaw, gripper]
        """
        # Preprocess inputs
        if isinstance(image, np.ndarray):
            image = Image.fromarray(image)

        inputs = self.processor(
            text=instruction,
            images=image,
            return_tensors="pt"
        ).to(self.device)

        # Generate action
        with torch.no_grad():
            outputs = self.model(**inputs)
            action_tokens = outputs.logits.argmax(dim=-1)

        # Decode action tokens to robot actions
        action = self.processor.decode_actions(action_tokens)[0]

        return np.array(action)  # Shape: (7,)

# Example usage
controller = OpenVLAController()

# Load camera image
image = Image.open("camera_feed.jpg")

# Get action
action = controller.predict_action(image, "pick up the red cup")
print(f"Predicted action: {action}")
# Output: [0.45, 0.12, 0.30, 0.0, 1.57, 0.0, 1.0]
#         [x,    y,    z,    r,   p,    y,    gripper_open]
```

---

## 13.4 ROS 2 Integration

### VLA Action Server

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import numpy as np

class VLAActionServer(Node):
    def __init__(self):
        super().__init__('vla_action_server')

        # VLA model
        self.vla = OpenVLAController()

        # ROS 2 interfaces
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb', self.image_callback, 10)
        self.cmd_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10)
        self.action_pub = self.create_publisher(
            Pose, '/robot/target_pose', 10)

        # State
        self.latest_image = None
        self.current_instruction = None

    def image_callback(self, msg):
        """Store latest camera image."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def command_callback(self, msg):
        """Execute VLA inference on voice command."""
        if self.latest_image is None:
            self.get_logger().warn('No camera image available')
            return

        instruction = msg.data
        self.get_logger().info(f'Executing: "{instruction}"')

        # VLA inference
        action = self.vla.predict_action(self.latest_image, instruction)

        # Publish target pose
        pose_msg = Pose()
        pose_msg.position.x = action[0]
        pose_msg.position.y = action[1]
        pose_msg.position.z = action[2]
        # orientation from roll, pitch, yaw (action[3:6])
        q = euler_to_quaternion(action[3], action[4], action[5])
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]

        self.action_pub.publish(pose_msg)
        self.get_logger().info(f'Published target: ({action[0]:.2f}, {action[1]:.2f}, {action[2]:.2f})')

def main():
    rclpy.init()
    node = VLAActionServer()
    rclpy.spin(node)
```

---

## 13.5 Fine-Tuning VLA

### Collect Custom Dataset

**Format** (HDF5):
```python
import h5py

# Create dataset
with h5py.File('custom_data.h5', 'w') as f:
    # Store trajectories
    for traj_idx in range(num_trajectories):
        grp = f.create_group(f'traj_{traj_idx}')

        grp.create_dataset('images', data=images)  # (T, H, W, 3)
        grp.create_dataset('actions', data=actions)  # (T, 7)
        grp.create_dataset('instruction', data=instruction.encode())
```

### Fine-Tuning Script

```python
from transformers import Trainer, TrainingArguments

# Load model
model = AutoModelForVision2Seq.from_pretrained("openvla/openvla-7b")

# Training config
training_args = TrainingArguments(
    output_dir="./openvla-finetuned",
    num_train_epochs=10,
    per_device_train_batch_size=8,
    learning_rate=1e-5,
    warmup_steps=100,
    logging_steps=10,
    save_steps=500,
    fp16=True,  # Mixed precision training
)

# Train
trainer = Trainer(
    model=model,
    args=training_args,
    train_dataset=train_dataset,
    eval_dataset=val_dataset,
)

trainer.train()
```

**Data Requirements**:
- Minimum: 100 demonstrations
- Recommended: 1,000+ demonstrations
- Best: 10,000+ demonstrations

---

## 13.6 End-to-End System

### Voice-Controlled Robot Pipeline

```
[User] "Pick up the red cup"
    ↓ Whisper (Speech → Text)
[Text] "Pick up the red cup"
    ↓ OpenVLA (Image + Text → Actions)
[Actions] [x, y, z, r, p, y, gripper]
    ↓ MoveIt 2 (Motion Planning)
[Joint Trajectory] θ(t)
    ↓ Robot Execution
[Physical Robot] Picks up cup ✅
```

### Full Pipeline Code

```python
import whisper
from openvla import OpenVLAController
from moveit_py import MoveItController

class VoiceControlledRobot:
    def __init__(self):
        # Components
        self.speech_model = whisper.load_model("base")
        self.vla = OpenVLAController()
        self.moveit = MoveItController()

    def execute_voice_command(self, audio_file, camera_image):
        """Execute robot action from voice command."""

        # Step 1: Speech → Text
        result = self.speech_model.transcribe(audio_file)
        instruction = result["text"]
        print(f"Heard: {instruction}")

        # Step 2: VLA → Action
        action = self.vla.predict_action(camera_image, instruction)
        print(f"VLA action: {action}")

        # Step 3: MoveIt → Trajectory
        trajectory = self.moveit.plan_to_pose(action[:6])  # position + orientation

        # Step 4: Execute
        self.moveit.execute(trajectory)

        # Step 5: Gripper control
        if action[6] > 0.5:
            self.moveit.open_gripper()
        else:
            self.moveit.close_gripper()

        print("Task complete!")

# Usage
robot = VoiceControlledRobot()
robot.execute_voice_command("command.wav", camera_image)
```

---

## 13.7 Assignment: Week 13

### Project: Deploy OpenVLA on Robot

**Scenario**: Table-top manipulation tasks

**Requirements**:

1. **VLA Inference** (40%)
   - Deploy OpenVLA (7B model)
   - Test on 10 tasks:
     - "pick the red block"
     - "put the cup in the bowl"
     - "grasp the tool"
   - Measure success rate

2. **ROS 2 Integration** (30%)
   - Subscribe to `/camera/rgb`
   - Subscribe to `/voice_command`
   - Publish to `/moveit/target_pose`

3. **Fine-Tuning** (30%)
   - Collect 50 demonstrations
   - Fine-tune OpenVLA
   - Compare: pre-trained vs fine-tuned accuracy

**Deliverables**:
- `vla_server.py` (ROS 2 node)
- Fine-tuning script
- Report: Success rates (before/after fine-tuning)
- Demo video: 3 successful tasks

**Bonus** (+10%): Multi-step tasks ("pick red cube and place it in the green box")

---

## 13.8 Key Takeaways

✅ **VLAs**: Single model for perception + planning + control

✅ **OpenVLA**: Open-source, 7B params, 800k trajectories

✅ **Generalization**: Works on novel objects and instructions

✅ **Fine-Tuning**: 100+ demos significantly improve performance

✅ **Future**: VLAs will power general-purpose robots 🤖

---

**Next**: [Chapter 14: LLM Task Planning →](./chapter14-llm-planning)
