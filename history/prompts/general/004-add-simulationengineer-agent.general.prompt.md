---
id: 004
title: Add SimulationEngineer Agent to Project
stage: general
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-calculator
user: HP
command: /sp.agents.add SimulationEngineer
labels: ["agent-creation", "simulation", "gazebo", "isaac-sim", "unity", "physics", "digital-twins"]
links:
  spec: specs/002-physical-ai-humanoid-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - .specify/agents/SimulationEngineer.md
  - .specify/agents/README.md
tests:
  - N/A (agent definition documents)
---

## Prompt

/sp.agents.add SimulationEngineer

## Response snapshot

**Agent Created**: SimulationEngineer (Domain-Focused Simulation Environment & Integration Agent)

**Purpose**: Specialized simulation agent responsible for generating simulation environments, world files, physics configurations, and integrating ROS 2 packages into various simulators (Gazebo, NVIDIA Isaac Sim, Unity). Creates realistic digital twins where learners can test robotics algorithms without physical hardware.

**Core Responsibilities (7 domains):**

1. **Gazebo/Ignition Simulation**
   - Generate SDF (Simulation Description Format) world files
   - Configure Gazebo plugins (sensors, actuators, physics)
   - Create environment models (buildings, obstacles, terrain)
   - Design lighting, shadows, and rendering settings
   - Implement sensor plugins (camera, lidar, IMU, force-torque)
   - Configure physics engines (ODE, Bullet, DART, Simbody)
   - Set up collision geometries and contact parameters

2. **NVIDIA Isaac Sim Integration**
   - Generate USD (Universal Scene Description) scenes
   - Configure Isaac Sim physics (PhysX, contact handling)
   - Implement synthetic data generation pipelines
   - Design domain randomization scenarios
   - Configure camera sensors (RGB, depth, semantic segmentation)
   - Set up ROS 2 bridge connections
   - Implement Isaac Sim extensions and custom scenarios

3. **Unity Robotics Hub Integration**
   - Create Unity scenes for human-robot interaction
   - Configure Unity Robotics Hub (ROS-TCP-Connector)
   - Implement Unity physics (Articulation Body, joints)
   - Design interactive environments and UI elements
   - Configure camera and sensor simulations
   - Set up topic/service routing between Unity and ROS 2
   - Implement C# scripts for robot control

4. **Digital Twin Creation**
   - Generate complete robot simulation environments
   - Synchronize simulation with real robot state
   - Implement sensor noise models for realism
   - Configure actuator dynamics and control interfaces
   - Design test scenarios for robotics algorithms
   - Validate simulation accuracy against physical systems

5. **Physics Configuration**
   - Configure gravity, time step, solver iterations
   - Set friction coefficients (static, dynamic, rolling)
   - Define restitution (bounciness) for materials
   - Configure damping (linear, angular) for joints
   - Set collision detection parameters (margin, algorithm)
   - Tune physics for stability vs. accuracy tradeoffs
   - Implement custom physics constraints

6. **Sensor Simulation**
   - Camera simulation (intrinsic/extrinsic, distortion)
   - Lidar/laser scanner (ray-based, point cloud generation)
   - IMU simulation (accelerometer, gyroscope, noise models)
   - Force-torque sensors (contact forces, joint torques)
   - Depth cameras (RGB-D, stereo, structured light)
   - GPS simulation (position, velocity, noise)

7. **Integration & Testing**
   - Integrate ROS 2 packages from ROS2Engineer into simulators
   - Create launch files for simulation startup
   - Generate test scenarios for algorithm validation
   - Configure headless simulation for CI/CD
   - Implement simulation speed control (real-time, faster, slower)
   - Design reproducible simulation experiments

**Domain Expertise:**

**Gazebo Classic & Ignition:**
- SDF format: XML-based scene description (worlds, models, plugins)
- Plugin system: ModelPlugin, WorldPlugin, SensorPlugin, VisualPlugin
- Physics engines: ODE (default), Bullet, DART, Simbody
- Rendering: OGRE (Classic), OGRE 2.x (Ignition)
- Sensors: Camera, depth camera, lidar, IMU, contact, force-torque
- ROS integration: gazebo_ros_pkgs for ROS 2

**NVIDIA Isaac Sim:**
- USD Pipeline: Omniverse USD for scene description
- PhysX: NVIDIA physics engine (high-fidelity, GPU-accelerated)
- Replicator: Synthetic data generation (domain randomization)
- Sensors: Cameras (RGB, depth, semantic, instance), lidar, contact
- ROS 2 Bridge: Bidirectional topic/service communication
- Isaac Gym: RL environment integration
- Extensions: Python/C++ extensions for custom functionality

**Unity Robotics:**
- Unity Editor: Scene creation, prefabs, asset management
- Articulation Body: Physics-based robot joints (Unity 2020.2+)
- ROS-TCP-Connector: Real-time ROS 2 communication
- URDF Importer: Convert ROS URDF to Unity articulation
- Sensor Simulation: Camera, lidar (custom implementations)
- C# Scripting: MonoBehaviour scripts for robot control
- Timeline: Animation and scenario sequencing

**Physics Simulation Fundamentals:**
- Numerical integration: Euler, Runge-Kutta, symplectic methods
- Collision detection: Broad phase (sweep-and-prune), narrow phase (GJK, EPA)
- Contact resolution: Penalty-based, constraint-based (LCP, PGS)
- Friction models: Coulomb friction, continuous friction approximation
- Stability: Time step selection, solver iterations, constraint stabilization

**Sensor Models:**
- Camera intrinsics: Focal length, principal point, distortion (radial, tangential)
- Lidar: Ray casting, range accuracy, angular resolution, scan rate
- IMU: Accelerometer/gyroscope bias, noise (white, random walk)
- Force-Torque: Measurement frame, noise, bias, filtering

**Boundaries:**
- ✅ Generate simulation world files (SDF, USD, Unity scenes)
- ✅ Configure physics engines and parameters
- ✅ Implement sensor simulations with realistic noise
- ✅ Integrate ROS 2 packages into simulators
- ✅ Create test scenarios and environments
- ✅ Design digital twins synchronized with real systems
- ✅ Generate synthetic data for ML training
- ❌ Generate ROS 2 code/packages (delegated to ROS2Engineer)
- ❌ Design control algorithms (receives from RoboticsExpert)
- ❌ Create URDF robot descriptions (receives from ROS2Engineer)
- ❌ Generate mathematical derivations (delegated to RoboticsExpert)
- ❌ Write educational narrative (delegated to ContentGeneration)

**Interaction Patterns:**
- **Upstream**: Receives requirements from BookPlanner; receives ROS 2 packages/URDF from ROS2Engineer; receives physics parameters from RoboticsExpert
- **Downstream**: Provides simulation examples to ContentGeneration; provides test results to ValidationAgent
- **Peer**: Collaborates with ROS2Engineer (packages→simulation), RoboticsExpert (physics models→sim parameters)

**Simulation Generation Workflow (7 phases):**
1. Requirements Analysis - Identify simulator, sensors, environment complexity
2. Environment Design - World layout, 3D models, lighting, test scenarios
3. Physics Configuration - Engine selection, global parameters, materials, collisions
4. Sensor Implementation - Camera, lidar, IMU, force-torque with noise models
5. ROS 2 Integration - Import URDF, attach plugins, configure topics/controllers
6. Launch Configuration - Create launch files, arguments, debugging tools
7. Testing & Validation - Verify stability, sensor data, control interfaces, performance

**Example Scenarios Documented:**
1. Gazebo Humanoid Balance Environment (Chapter 10 - ZMP testing)
2. Isaac Sim Synthetic Data Generation (Chapter 7 - perception training)
3. Unity Human-Robot Interaction (Chapter 6 - HRI demonstration)

**Simulation Best Practices Enforced:**

**Physics Accuracy vs. Stability:**
- Real-time factor: Target 1.0 for interactive use
- Time step: Smaller (0.001s) for stability, larger (0.01s) for speed
- Solver iterations: More (50-100) for complex contact, fewer (20) for simple
- Collision margins: Small (0.001m) for accuracy, larger (0.01m) for stability

**Sensor Realism:**
- Camera: Include lens distortion, motion blur, rolling shutter
- Lidar: Model range noise, reflectivity, occlusion, beam divergence
- IMU: Include bias drift, random walk, temperature sensitivity
- Force-Torque: Add measurement noise, quantization, frame offsets

**Performance Optimization:**
- Simplified collision: Use primitive shapes (boxes, cylinders) vs. meshes
- Level of Detail (LOD): Reduce mesh complexity based on distance
- Culling: Disable rendering for objects outside view
- Spatial partitioning: Use octrees, k-d trees for collision detection
- GPU acceleration: Offload physics to GPU (PhysX, Isaac Sim)

**Reproducibility:**
- Random seeds: Set fixed seeds for deterministic behavior
- Logging: Record simulation state, sensor data, events
- Versioning: Document simulator version, plugin versions, asset versions
- Containerization: Provide Docker images for consistent environment

**File Formats:**
- SDF (Simulation Description Format) for Gazebo
- USD (Universal Scene Description) for Isaac Sim
- Unity Prefab structure for Unity scenes
- Launch files (Python) for simulation startup
- YAML configs for physics parameters, sensor noise

**Constitution Compliance:**
- **Principle VI (PRIMARY)**: Simulation examples specify environment setup, must be reproducible
- **Principle I**: Physics parameters based on real-world values (from RoboticsExpert)
- **Principle VII**: Simulation examples launch successfully, physics stability verified, sensor data validated
- **Principle III**: World files modular, reusable environments across chapters
- **Principle IV**: Consistent coordinate system, naming, physics parameters

**Knowledge Domains by Chapter:**
- Ch 5: Gazebo (SDF worlds, plugins, physics, sensors, URDF import)
- Ch 6: Unity (scenes, ROS-TCP-Connector, C# scripts, interactive UI)
- Ch 7: Isaac Sim (USD scenes, PhysX, Replicator, synthetic data, ROS 2 bridge)
- Ch 8: Isaac ROS (VSLAM/navigation environments, sensor configs, benchmarks)
- Ch 9-11: Kinematics/Control/Manipulation (FK/IK test environments, balance scenarios, manipulation tasks)
- Ch 12-13: Voice/VLA (interactive environments, object layouts, language grounding scenarios)

**Output Formats:**
- Simulation package structure (worlds/, launch/, config/, scripts/, models/, docs/, test/)
- Simulation metadata (YAML with simulator, world file, robot, physics, sensors, dependencies)
- Complete examples (worlds, launch files, docs, test scenarios)

**Human-in-the-Loop Triggers:**
1. Simulator selection ambiguity (multiple suitable options)
2. Physics accuracy vs. performance (tradeoff decision)
3. Environment complexity (level of detail)
4. Sensor configuration (multiple valid setups)
5. License constraints (proprietary assets/simulators)
6. Hardware requirements (high-end GPU accessibility)

**Performance Metrics:**
- Launch Success Rate: 100% (simulation launches without errors)
- Real-Time Factor: ≥0.8 for interactive simulations
- Physics Stability: Zero catastrophic failures (exploding robots)
- Sensor Data Validity: 100% (correct topics, message types, reasonable ranges)
- Integration Success: ROS 2 packages integrate without modification
- Reproducibility: Fixed seed produces identical results

**Registry Updates:**
- Added SimulationEngineer to active agents
- Updated ecosystem architecture diagram (RoboticsExpert provides physics parameters to SimulationEngineer; ROS2Engineer provides ROS 2 packages to SimulationEngineer; SimulationEngineer provides simulation worlds/examples to ContentGeneration and test results to ValidationAgent)
- Removed planned SimulationAgent (now replaced by SimulationEngineer)

## Outcome

- ✅ Impact: Created comprehensive simulation environment authority; established multi-simulator framework (Gazebo, Isaac Sim, Unity); enables digital twin creation for testing robotics algorithms without physical hardware; provides realistic sensor simulation with noise models
- 🧪 Tests: Simulation generation workflow (7 phases); validation includes launch success, physics stability, sensor data validity; example scenarios demonstrate Gazebo balance environment, Isaac Sim synthetic data, Unity HRI scene
- 📁 Files: Created `.specify/agents/SimulationEngineer.md` (comprehensive agent definition, ~22KB) and updated `.specify/agents/README.md` (registry with updated ecosystem, removed redundant SimulationAgent)
- 🔁 Next prompts: SimulationEngineer ready for invocation during simulation example generation; will create Gazebo worlds for Chapters 5, 9-11; Isaac Sim scenes for Chapters 7-8; Unity scenes for Chapter 6; recommend testing with Gazebo humanoid balance environment
- 🧠 Reflection: SimulationEngineer agent design prioritizes Constitution Principle VI (Code & Simulation Standards) with Principle I (physics accuracy). Agent receives ROS 2 packages from ROS2Engineer and physics parameters from RoboticsExpert, integrates into simulation environments. Collaboration pattern: RoboticsExpert provides physics models → ROS2Engineer implements URDF/controllers → SimulationEngineer creates worlds around them → ValidationAgent verifies stability/accuracy → ContentGeneration integrates into chapters. Multi-simulator support (Gazebo, Isaac, Unity) provides flexibility for different educational scenarios.

## Evaluation notes (flywheel)

- Failure modes observed: None; agent creation successful
- Graders run and results (PASS/FAIL):
  - PASS: Agent definition comprehensive with all 7 responsibility domains
  - PASS: Multi-simulator expertise documented (Gazebo, Isaac Sim, Unity)
  - PASS: Clear boundaries (generates simulation worlds; delegates ROS 2 code/URDF/algorithms)
  - PASS: Best practices enforced (physics accuracy vs. stability, sensor realism, performance optimization, reproducibility)
  - PASS: Interaction patterns documented (upstream from BookPlanner/ROS2Engineer/RoboticsExpert; downstream to ContentGeneration/ValidationAgent)
  - PASS: Constitution compliance verified (Principle VI as PRIMARY with Principle I for physics accuracy)
  - PASS: Knowledge domains mapped to chapters (5-13)
  - PASS: Physics simulation fundamentals documented (numerical integration, collision detection, contact resolution, friction models)
  - PASS: Example scenarios demonstrate workflow (Gazebo balance, Isaac synthetic data, Unity HRI)
  - PASS: File formats specified (SDF, USD, Unity prefabs, launch files, YAML configs)
- Prompt variant (if applicable): N/A (initial agent creation)
- Next experiment (smallest change to try): Test SimulationEngineer by generating Gazebo humanoid balance world from ROS2Engineer humanoid URDF; validate SDF world file syntax; verify Gazebo launch succeeds; assess physics stability (no jittering/explosions); measure integration workflow effectiveness (ROS2Engineer URDF → SimulationEngineer world → Gazebo simulation → sensor topics published correctly)
