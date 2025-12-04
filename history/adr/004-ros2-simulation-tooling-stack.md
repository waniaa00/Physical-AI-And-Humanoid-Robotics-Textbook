# ADR-004: ROS 2 and Multi-Simulator Tooling Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-04
- **Feature:** Physical AI & Humanoid Robotics Book
- **Context:** Robotics education requires executable code examples and realistic simulation environments. Learners need hands-on experience with industry-standard tools (ROS 2), physics-accurate simulators (Gazebo, Isaac Sim), and accessible alternatives (Unity). Code examples must run on learner hardware (mid-range GPU) and integrate across tools.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ Long-term - defines robotics platform for all practical examples (5-10 year lifespan)
     2) Alternatives: ✅ Multiple options (ROS 1, custom framework, simulation-only, real robots)
     3) Scope: ✅ Cross-cutting - affects 12/15 chapters, code examples, simulation integration
-->

## Decision

Adopt **ROS 2 + Multi-Simulator Stack** with version pinning and Docker alternatives:

**Robotics Framework:**
- **ROS 2 Distribution**: Humble Hawksbill (LTS until May 2027)
- **Platform**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Language**: Python 3.10+ (rclpy), C++ optional for performance examples
- **Build System**: colcon (ROS 2 standard)
- **Message Definitions**: geometry_msgs, sensor_msgs, std_msgs (ROS 2 common interfaces)

**Simulation Environments** (3 simulators for pedagogical diversity):
1. **Gazebo Classic 11 / Ignition Fortress**
   - Use Case: Physics simulation, URDF visualization, sensor simulation
   - Integration: gazebo_ros bridge (ROS 2 topics/services)
   - Pros: Open-source, lightweight, wide ROS 2 support

2. **Unity 2022 LTS + Robotics Hub**
   - Use Case: Photorealistic rendering, human-robot interaction, game-like scenarios
   - Integration: ROS-TCP-Connector (bidirectional message passing)
   - Pros: Visual quality, accessible to game developers, free for education

3. **NVIDIA Isaac Sim 2023.1+**
   - Use Case: GPU-accelerated physics (PhysX), synthetic data generation (Replicator), Isaac ROS perception
   - Integration: Isaac ROS 2 bridge (native support)
   - Pros: High-fidelity physics, domain randomization, ML training data

**Code Testing & Validation:**
- **Unit Testing**: pytest (Python), gtest (C++)
- **Integration Testing**: launch_testing (ROS 2 launch file validation)
- **Code Quality**: ament_lint (copyright, PEP 8, xmllint for URDF)
- **CI/CD**: GitHub Actions with ROS 2 Humble Docker container

**Deployment Alternatives:**
- **Native Install**: Ubuntu 22.04 + ROS 2 Humble (recommended for performance)
- **Docker**: ros:humble-desktop-full image (for Windows/Mac learners)
- **Cloud**: AWS RoboMaker / Google Cloud Robotics (for GPU-intensive Isaac Sim)

## Consequences

### Positive

- **Industry Standard**: ROS 2 Humble is production-ready (Tesla, Boston Dynamics use ROS 2)
- **Long-Term Support**: Humble LTS until 2027 → content remains relevant for 2+ years
- **Ecosystem**: Mature package ecosystem (Nav2, ros2_control, MoveIt 2)
- **Multi-Simulator**: 3 simulators expose learners to different use cases (physics vs visuals vs ML data)
- **Cross-Platform**: Docker enables Windows/Mac learners to run ROS 2 examples
- **Testing Infrastructure**: colcon test + GitHub Actions automate code validation (Gate 2 in ADR-003)
- **Isaac Sim Advantage**: GPU-accelerated physics enables realistic humanoid simulation (walking, grasping)

### Negative

- **Version Lock-In**: Humble LTS expires 2027 → migration to Jazzy/Iron required in future
- **Complexity**: 3 simulators increase setup burden for learners (mitigated by setup guides per simulator)
- **GPU Requirement**: Isaac Sim requires NVIDIA GPU (RTX 3060+ recommended) → not accessible to all learners (mitigated by cloud alternatives)
- **Isaac Sim Licensing**: Free for education but requires NVIDIA account → barrier for some learners
- **Docker Overhead**: Virtualization adds latency for real-time control (mitigated by native install recommendation)
- **Unity Learning Curve**: C# scripting required for custom Unity behaviors (mitigated by provided templates)

## Alternatives Considered

**Alternative A: ROS 1 Noetic**
- **Approach**: Use ROS 1 (last LTS, EOL 2025) instead of ROS 2
- **Rejected Because**:
  - End of life in 2025 → content outdated immediately
  - No DDS middleware (ROS 2's key feature for distributed systems)
  - Industry moving to ROS 2 (new robotics companies use ROS 2 exclusively)
  - Limited Python 3 support (ROS 1 primarily Python 2.7)

**Alternative B: Custom Robotics Framework**
- **Approach**: Build simplified robotics library for educational purposes
- **Rejected Because**:
  - Not transferable to industry (learners need ROS 2 experience for jobs)
  - Reinventing the wheel (ROS 2 solves communication, serialization, tooling)
  - No ecosystem (no pre-built navigation, manipulation packages)
  - Maintenance burden (framework must be maintained indefinitely)

**Alternative C: Gazebo Only (No Unity/Isaac Sim)**
- **Approach**: Standardize on Gazebo for all simulation examples
- **Rejected Because**:
  - Gazebo lacks photorealistic rendering (Unity superior for HRI scenarios)
  - Gazebo lacks GPU physics (Isaac Sim faster for large-scale sim)
  - Single simulator limits pedagogical range (different tools for different problems)
  - Missing Isaac ROS perception examples (SLAM, object detection)

**Alternative D: Real Robot Hardware (No Simulation)**
- **Approach**: Require learners to purchase physical robots (e.g., TurtleBot, Unitree Go1)
- **Rejected Because**:
  - Cost barrier ($500-$5,000+ per robot) excludes most learners
  - Scalability issues (not all learners have space for robot)
  - Safety concerns (bipedal robots can fall, damage occurs)
  - Iteration speed (simulation enables rapid testing, real robots slow)

**Alternative E: Web-Based Simulation (e.g., Robotics Simulator in Browser)**
- **Approach**: Use WebGL-based simulator (e.g., RobotJS, Webots)
- **Rejected Because**:
  - Limited physics accuracy (no PhysX, simplified dynamics)
  - No ROS 2 integration (cannot test real ROS 2 code)
  - Browser performance constraints (complex robots lag)
  - Not industry-standard (no job market relevance)

## References

- Feature Spec: [specs/002-physical-ai-humanoid-robotics-book/spec.md](../../specs/002-physical-ai-humanoid-robotics-book/spec.md) (FR-021 to FR-025: Simulation & Code Requirements)
- Implementation Plan: [specs/002-physical-ai-humanoid-robotics-book/plan.md](../../specs/002-physical-ai-humanoid-robotics-book/plan.md) (Section 0.2-0.3: ROS 2 & Simulation Tools Validation, Section 2.4-2.5: Workspace & Simulation Setup)
- ROS 2 Humble Docs: https://docs.ros.org/en/humble/
- Isaac Sim Docs: https://docs.omniverse.nvidia.com/isaacsim/latest/
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- Related ADRs: ADR-001 (Multi-Agent Architecture - ROS2Engineer, SimulationEngineer, IsaacExpert), ADR-003 (Quality Gates - Code Testing)
