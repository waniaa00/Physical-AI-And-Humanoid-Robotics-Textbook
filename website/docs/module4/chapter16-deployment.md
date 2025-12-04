---
sidebar_position: 4
title: "Chapter 16: Deployment"
---

# Chapter 16: Production Deployment

**Week 16 | Module 4**

## 16.1 Containerization with Docker

```dockerfile
FROM ros:humble
RUN apt-get update && apt-get install -y python3-pip
COPY . /workspace
RUN pip install -r requirements.txt
CMD ["ros2", "launch", "robot_bringup", "robot.launch.py"]
```

## 16.2 Edge Deployment
- NVIDIA Jetson Orin
- Raspberry Pi 4

## 16.3 Final Project: Home Assistant Robot

**Deliverables**:
1. Navigate 3-room apartment
2. Fetch objects on voice command
3. Manipulation of doors/drawers
4. Safe human avoidance

**Demo Video**: 5-minute showcase of all capabilities

---

🎉 **Congratulations! You've completed the Physical AI & Humanoid Robotics course!**
