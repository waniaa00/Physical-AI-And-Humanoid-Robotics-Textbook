---
sidebar_position: 2
title: "Chapter 14: LLM Task Planning"
---

# Chapter 14: LLM-Based Task Planning

**Week 14 | Module 4**

## 14.1 LLM as Task Planner

```python
import anthropic

client = anthropic.Anthropic(api_key="...")
response = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    messages=[{"role": "user", "content": "Plan steps to make coffee"}]
)
```

## 14.2 Grounding Language to Actions
Map high-level plans to robot primitives

## 14.3 Assignment
Build voice-controlled robot with Whisper + Claude + ROS 2

---
**Next**: [Chapter 15: System Integration →](./chapter15-integration)
