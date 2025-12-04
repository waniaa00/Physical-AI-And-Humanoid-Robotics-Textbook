---
sidebar_position: 2
title: "Chapter 6: NVIDIA Isaac Sim"
---

# Chapter 6: GPU-Accelerated Simulation

**Week 6 | Module 2: Simulation & Perception**

:::info Learning Objectives
- ✅ Set up NVIDIA Isaac Sim
- ✅ Generate synthetic training data with Replicator
- ✅ Implement domain randomization
- ✅ Integrate Isaac ROS 2
:::

---

## 6.1 Why Isaac Sim?

### Key Advantages
- **GPU Physics**: 10-100x faster than CPU (PhysX 5)
- **Ray-Traced Rendering**: Photorealistic images
- **Replicator**: Auto-generate labeled datasets
- **Robot Library**: Pre-built UR10, Franka, ANYmal, etc.

---

## 6.2 Installation

### System Requirements
- **GPU**: RTX 3060+ (6GB VRAM minimum)
- **OS**: Ubuntu 22.04 / Windows 11
- **RAM**: 32GB recommended

### Install via Omniverse

```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/...

# Install Isaac Sim 2023.1.1
# Via Launcher: Exchange → Isaac Sim → Install
```

---

## 6.3 Synthetic Data Generation

### Replicator: Domain Randomization

```python
import omni.replicator.core as rep

# Create camera
camera = rep.create.camera(position=(2, 2, 1))

# Randomize lighting
def randomize_lights():
    lights = rep.create.light(
        light_type="Sphere",
        temperature=rep.distribution.uniform(3000, 6500),
        intensity=rep.distribution.uniform(10000, 50000),
        position=rep.distribution.uniform((-5, -5, 3), (5, 5, 7))
    )
    return lights.node

# Randomize object textures
def randomize_materials():
    cube = rep.create.cube(semantics=[('class', 'cube')])
    with cube:
        rep.modify.pose(
            position=rep.distribution.uniform((-1, -1, 0), (1, 1, 2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )
    return cube.node

# Register randomizers
rep.randomizer.register(randomize_lights)
rep.randomizer.register(randomize_materials)

# Generate 1,000 frames
with rep.trigger.on_frame(num_frames=1000):
    rep.randomizer.randomize_lights()
    rep.randomizer.randomize_materials()

# Capture RGB + Depth + Segmentation
rp = rep.AnnotatorRegistry.get_annotator("rgb")
depth = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
seg = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")

# Run and save
rep.orchestrator.run()
```

### Output Format (COCO JSON)

```json
{
  "images": [
    {"id": 1, "file_name": "rgb_0001.png", "width": 1024, "height": 768}
  ],
  "annotations": [
    {"id": 1, "image_id": 1, "category_id": 1, "bbox": [100, 150, 200, 180]}
  ],
  "categories": [
    {"id": 1, "name": "cube"}
  ]
}
```

---

## 6.4 Isaac ROS 2 Integration

### Publish Camera Feed

```python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import omni.isaac.core.utils.numpy.rotations as rotations

class IsaacCameraPublisher(Node):
    def __init__(self):
        super().__init__('isaac_camera')
        self.publisher = self.create_publisher(Image, '/camera/rgb', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_frame)

        # Isaac Sim camera setup
        from omni.isaac.sensor import Camera
        self.camera = Camera("/World/Camera")

    def publish_frame(self):
        # Get frame from Isaac Sim
        rgb = self.camera.get_rgba()[:, :, :3]  # Drop alpha

        # Convert to ROS Image
        msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"

        self.publisher.publish(msg)
```

---

## 6.5 Assignment: Week 6

### Project: Generate 1,000 Labeled Training Images

**Scenario**: Bin picking with cubes, cylinders, spheres

**Requirements**:
1. Domain randomization:
   - Object positions (random in 1m³ bin)
   - Lighting (3000-6500K, varying intensity)
   - Textures (10 different materials)
   - Camera pose (slight variations)

2. Annotations:
   - 2D bounding boxes
   - Semantic segmentation masks
   - Depth maps

3. Export:
   - COCO JSON format
   - Verify with labelImg

**Deliverables**:
- Replicator script
- 1,000 images + annotations
- Sample visualization (5 images with bboxes)

---

**Next**: [Chapter 7: Computer Vision →](./chapter7-computer-vision)
