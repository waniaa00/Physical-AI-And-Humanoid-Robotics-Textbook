---
sidebar_position: 3
title: "Chapter 7: Computer Vision"
---

# Chapter 7: Computer Vision for Robotics

**Week 7 | Module 2**

## 7.1 Object Detection

### YOLO Integration
```python
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
results = model(image)
boxes = results[0].boxes
```

## 7.2 3D Pose Estimation
- PnP algorithm
- FoundationPose
- ArUco markers

## 7.3 Assignment
Build grasp pose estimator using YOLO + depth camera

---
**Next**: [Chapter 8: SLAM & Navigation →](./chapter8-slam-navigation)
